/*
 * Copyright (C) 2015, Jonathan Meyer
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Tokyo Opensource Robotics Kyokai Association. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// copied from https://github.com/ros-industrial/robotiq/blob/jade-devel/robotiq_ethercat/src/ethercat_manager.cpp


#include "ethercat_manager/ethercat_manager.h"

#include <unistd.h>
#include <stdio.h>
#include <time.h>

#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

namespace
{
static const unsigned THREAD_SLEEP_TIME = 1000; // 1 ms
static const unsigned EC_TIMEOUTMON = 500;
static const int NSEC_PER_SECOND = 1e+9;
void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
      tick.tv_nsec -= NSEC_PER_SECOND;
      tick.tv_sec++;
    }
}

void handleErrors()
{
  /* one ore more slaves are not responding */
  ec_group[0].docheckstate = FALSE;
  ec_readstate();
  for (int slave = 1; slave <= ec_slavecount; slave++)
  {
    if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
    {
      ec_group[0].docheckstate = TRUE;
      if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
      {
        fprintf(stderr, "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
      {
        fprintf(stderr, "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave);
      }
      else if(ec_slave[slave].state > 0)
      {
        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
        {
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE : slave %d reconfigured\n",slave);
        }
      }
      else if(!ec_slave[slave].islost)
      {
        /* re-check state */
        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if (!ec_slave[slave].state)
        {
          ec_slave[slave].islost = TRUE;
          fprintf(stderr, "ERROR : slave %d lost\n",slave);
          throw std::runtime_error("ERROR : slave lost");
        }
      }
    }
    if (ec_slave[slave].islost)
    {
      if(!ec_slave[slave].state)
      {
        if (ec_recover_slave(slave, EC_TIMEOUTMON))
        {
          ec_slave[slave].islost = FALSE;
          printf("MESSAGE : slave %d recovered\n",slave);
        }
      }
      else
      {
        ec_slave[slave].islost = FALSE;
        printf("MESSAGE : slave %d found\n",slave);
      }
    }
  }
}

void cycleWorker(boost::mutex& mutex, bool& stop_flag)
{
  // 1ms in nanoseconds
  double period = THREAD_SLEEP_TIME * 1000;
  // get curren ttime
  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  timespecInc(tick, period);
  while (!stop_flag)
  {
    int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    int sent, wkc;
    {
      boost::mutex::scoped_lock lock(mutex);
      sent = ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
    }
    
    if (wkc < expected_wkc)
    {
      handleErrors();
    }
    // check overrun
    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
    if (overrun_time > 0.0)
      {
	      //fprintf(stderr, "  overrun: %f\n", overrun_time);
      }
    //usleep(THREAD_SLEEP_TIME);
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    timespecInc(tick, period);
    //printf("%24.12f %14.10f [msec] %02x %02x %02x %02x\n", tick.tv_sec+double(tick.tv_nsec)/NSEC_PER_SECOND, overrun_time*1000, ec_slave[1].outputs[24], ec_slave[1].outputs[23], ec_slave[1].outputs[22], ec_slave[1].outputs[21]);
  }
}

} // end of anonymous namespace


namespace ethercat {

EtherCatManager::EtherCatManager(const std::string& ifname)
  : ifname_(ifname),
    num_clients_(0),
    stop_flag_(false)
{
  if (initSoem(ifname))
  {
    cycle_thread_ = boost::thread(cycleWorker,
                                  boost::ref(iomap_mutex_),
                                  boost::ref(stop_flag_));
  }
  else
 {
   // construction failed
   throw EtherCatError("Could not initialize SOEM");
 }
}

EtherCatManager::~EtherCatManager()
{
  stop_flag_ = true;

  // Request init operational state for all slaves
  ec_slave[0].state = EC_STATE_INIT;

  /* request init state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();
  cycle_thread_.join();
}

#define IF_MINAS(_ec_slave) (((int)_ec_slave.eep_man == 0x066f) && ((((0xf0000000&(int)ec_slave[cnt].eep_id)>>28) == 0x5) || (((0xf0000000&(int)ec_slave[cnt].eep_id)>>28) == 0xD)))
bool EtherCatManager::initSoem(const std::string& ifname) {
  // Copy string contents because SOEM library doesn't
  // practice const correctness
  const static unsigned MAX_BUFF_SIZE = 1024;
  char buffer[MAX_BUFF_SIZE];
  size_t name_size = ifname_.size();
  if (name_size > sizeof(buffer) - 1)
  {
    fprintf(stderr, "Ifname %s exceeds maximum size of %u bytes\n", ifname_.c_str(), MAX_BUFF_SIZE);
    return false;
  }
  std::strncpy(buffer, ifname_.c_str(), MAX_BUFF_SIZE);

  printf("Initializing etherCAT master\n");

  if (!ec_init(buffer))
  {
    fprintf(stderr, "Could not initialize ethercat driver\n");
    return false;
  }

  /* find and auto-config slaves */
  if (ec_config_init(FALSE) <= 0)
  {
    fprintf(stderr, "No slaves found on %s\n", ifname_.c_str());
    return false;
  }

  printf("SOEM found and configured %d slaves\n", ec_slavecount);
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
  {
    // MINAS-A5B Serial Man = 066Fh, ID = [5/D]****[0/4/8][0-F]*
    printf(" Man: %8.8x ID: %8.8x Rev: %8.8x %s\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev, IF_MINAS(ec_slave[cnt])?" MINAS Drivers":"");
    if(IF_MINAS(ec_slave[cnt])) {
      num_clients_++;
    }
  }
  printf("Found %d MINAS Drivers\n", num_clients_);


  /*
    SET PDO maping 4    SX-DSV02470 p.52
			Index	  Size(bit)	Name
    RxPDO (1603h)	6040h 00h 16 Controlword
			6060h 00h  8 Modes of operation
			6071h 00h 16 Target Torque
			6072h 00h 16 Max torque
			607Ah 00h 32 Target Position
			6080h 00h 32 Max motor speed
			60B8h 00h 16 Touch probe function
			60FFh 00h 32 Target Velocity
    TxPDO (1A03h)
			603Fh 00h 16 Error code
			6041h 00h 16 Statusword
			6061h 00h  8 Modes of operation display
			6064h 00h 32 Position actual value
			606Ch 00h 32 Velocity actual value
			6077h 00h 16 Torque actual value
			60B9h 00h 16 Touch probe status
			60BAh 00h 32 Touch probe pos1 pos val
			60FDh 00h 32 Digital inputs
   */
  if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_PRE_OP)
    {
      fprintf(stderr, "Could not set EC_STATE_PRE_OP\n");
      return false;
    }

  // extend PDO mapping 4 see p. 53 of SX-DSV02470
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      if (! IF_MINAS(ec_slave[cnt])) continue;
      int ret = 0, l;
      uint8_t num_entries;
      l = sizeof(num_entries);
      ret += ec_SDOread(cnt, 0x1603, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
      printf("len = %d\n", num_entries);
      num_entries = 0;
      ret += ec_SDOwrite(cnt, 0x1603, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
      // add position offset (60B0 / 00h / I32)
      uint32_t mapping;
      mapping = 0x60B00020;
      ret += ec_SDOwrite(cnt, 0x1603, 0x09, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
      //
      num_entries = 9;
      ret += ec_SDOwrite(cnt, 0x1603, 0x00, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
      ret += ec_SDOread(cnt, 0x1603, 0x00, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
        /*
      //uint32_t mapping;
      mapping = 0x60FE0120;
      ret += ec_SDOwrite(cnt, 0x1a03, 0x0a, FALSE, sizeof(mapping), &mapping, EC_TIMEOUTRXM);
      //
      num_entries = 10;
      l = sizeof(num_entries);
      ret += ec_SDOwrite(cnt, 0x1a03, 0x0a, FALSE, sizeof(num_entries), &num_entries, EC_TIMEOUTRXM);
      ret += ec_SDOread(cnt, 0x1a03, 0x0a, FALSE, &l, &num_entries, EC_TIMEOUTRXM);
    */
      printf("len = %d\n", num_entries);
    }

  // use PDO mapping 4
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      if (! IF_MINAS(ec_slave[cnt])) continue;
      int ret = 0, l;
      uint8_t num_pdo ;
      //uint8_t num_entries;
      // set 0 change PDO mapping index
      num_pdo = 0;
      ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      // set to default PDO mapping 4
      uint16_t idx_rxpdo = 0x1603;
      ret += ec_SDOwrite(cnt, 0x1c12, 0x01, FALSE, sizeof(idx_rxpdo), &idx_rxpdo, EC_TIMEOUTRXM);
      // set number of assigned PDOs
      num_pdo = 1;
      ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      printf("RxPDO mapping object index %d = %04x ret=%d\n", cnt, idx_rxpdo, ret);

      // set 0 change PDO mapping index
      num_pdo = 0;
      ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      // set to default PDO mapping 4
      uint16_t idx_txpdo = 0x1a03;
      ret += ec_SDOwrite(cnt, 0x1c13, 0x01, FALSE, sizeof(idx_txpdo), &idx_txpdo, EC_TIMEOUTRXM);
      // set number of assigned PDOs
      num_pdo = 1;
      ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
      printf("TxPDO mapping object index %d = %04x ret=%d\n", cnt, idx_txpdo, ret);


    }

  // configure IOMap
  int iomap_size = ec_config_map(iomap_);
  printf("SOEM IOMap size: %d\n", iomap_size);

  // locates dc slaves - ???
  ec_configdc();

  // '0' here addresses all slaves
  if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_SAFE_OP)
  {
    fprintf(stderr, "Could not set EC_STATE_SAFE_OP\n");
    return false;
  }

  /*
      This section attempts to bring all slaves to operational status. It does so
      by attempting to set the status of all slaves (ec_slave[0]) to operational,
      then proceeding through 40 send/recieve cycles each waiting up to 50 ms for a
      response about the status.
  */
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  ec_writestate(0);
  int chk = 40;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  if(ec_statecheck(0,EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
  {
    fprintf(stderr, "OPERATIONAL state not set, exiting\n");
    return false;
  }

  ec_readstate();
  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      if (! IF_MINAS(ec_slave[cnt])) continue;
      printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
	     cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
	     ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
      if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
      printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
	     (ec_slave[cnt].activeports & 0x02) > 0 ,
	     (ec_slave[cnt].activeports & 0x04) > 0 ,
	     (ec_slave[cnt].activeports & 0x08) > 0 );
      printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
    }

  for( int cnt = 1 ; cnt <= ec_slavecount ; cnt++)
    {
      if (! IF_MINAS(ec_slave[cnt])) continue;
      int ret = 0, l;
      uint16_t sync_mode;
      uint32_t cycle_time;
      uint32_t minimum_cycle_time;
      uint32_t sync0_cycle_time;
      l = sizeof(sync_mode);
      ret += ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &sync_mode, EC_TIMEOUTRXM);
      l = sizeof(cycle_time);
      ret += ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &cycle_time, EC_TIMEOUTRXM);
      l = sizeof(minimum_cycle_time);
      ret += ec_SDOread(cnt, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time, EC_TIMEOUTRXM);
      l = sizeof(sync0_cycle_time);
      ret += ec_SDOread(cnt, 0x1c32, 0x0a, FALSE, &l, &sync0_cycle_time, EC_TIMEOUTRXM);
      printf("PDO syncmode %02x, cycle time %d ns (min %d), sync0 cycle time %d ns, ret = %d\n", sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time, ret);
    }

  printf("\nFinished configuration successfully\n");
  return true;
}

int EtherCatManager::getNumClients() const
{
  return num_clients_;
}

void EtherCatManager::write(int slave_no, uint8_t channel, uint8_t value)
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  ec_slave[slave_no].outputs[channel] = value;
}

uint8_t EtherCatManager::readInput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Ibits) {
    fprintf(stderr, "ERROR : channel(%d) is larget thatn Input bits (%d)\n", channel*8, ec_slave[slave_no].Ibits);
    exit(1);
  }
  return ec_slave[slave_no].inputs[channel];
}

uint8_t EtherCatManager::readOutput(int slave_no, uint8_t channel) const
{
  boost::mutex::scoped_lock lock(iomap_mutex_);
  if (slave_no > ec_slavecount) {
    fprintf(stderr, "ERROR : slave_no(%d) is larger than ec_slavecount(%d)\n", slave_no, ec_slavecount);
    exit(1);
  }
  if (channel*8 >= ec_slave[slave_no].Obits) {
    fprintf(stderr, "ERROR : channel(%d) is larget thatn Output bits (%d)\n", channel*8, ec_slave[slave_no].Obits);
    exit(1);
  }
  return ec_slave[slave_no].outputs[channel];
}

template <typename T>
uint8_t EtherCatManager::writeSDO(int slave_no, uint16_t index, uint8_t subidx, T value) const
{
  int ret;
  ret = ec_SDOwrite(slave_no, index, subidx, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE);
  return ret;
}

template <typename T>
T EtherCatManager::readSDO(int slave_no, uint16_t index, uint8_t subidx) const
{
  int ret, l;
  T val;
  l = sizeof(val);
  ret = ec_SDOread(slave_no, index, subidx, FALSE, &l, &val, EC_TIMEOUTRXM);
  if ( ret <= 0 ) { // ret = Workcounter from last slave response
    fprintf(stderr, "Failed to read from ret:%d, slave_no:%d, index:0x%04x, subidx:0x%02x\n", ret, slave_no, index, subidx);
  }
  return val;
}

template uint8_t EtherCatManager::writeSDO<char> (int slave_no, uint16_t index, uint8_t subidx, char value) const;
template uint8_t EtherCatManager::writeSDO<int> (int slave_no, uint16_t index, uint8_t subidx, int value) const;
template uint8_t EtherCatManager::writeSDO<short> (int slave_no, uint16_t index, uint8_t subidx, short value) const;
template uint8_t EtherCatManager::writeSDO<long> (int slave_no, uint16_t index, uint8_t subidx, long value) const;
template uint8_t EtherCatManager::writeSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx, unsigned char value) const;
template uint8_t EtherCatManager::writeSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx, unsigned int value) const;
template uint8_t EtherCatManager::writeSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx, unsigned short value) const;
template uint8_t EtherCatManager::writeSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx, unsigned long value) const;

template char EtherCatManager::readSDO<char> (int slave_no, uint16_t index, uint8_t subidx) const;
template int EtherCatManager::readSDO<int> (int slave_no, uint16_t index, uint8_t subidx) const;
template short EtherCatManager::readSDO<short> (int slave_no, uint16_t index, uint8_t subidx) const;
template long EtherCatManager::readSDO<long> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned char EtherCatManager::readSDO<unsigned char> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned int EtherCatManager::readSDO<unsigned int> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned short EtherCatManager::readSDO<unsigned short> (int slave_no, uint16_t index, uint8_t subidx) const;
template unsigned long EtherCatManager::readSDO<unsigned long> (int slave_no, uint16_t index, uint8_t subidx) const;

}

