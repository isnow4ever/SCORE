/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test -i [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 */

#include <stdio.h>
#include <ethercat_manager/ethercat_manager.h>
#include <minas_control/minas_client.h>
#include <getopt.h>
#include <time.h>

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

void timespecInc(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}

int main(int argc, char *argv[])
{
    int operation_mode = 0x08; // (pp) position profile mode
    std::string ifname;
	ifname = "eth0";
    printf("MINAS Simple Test using SOEM (Simple Open EtherCAT Master)\n");
	/*
    while (1) {
        static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"position_mode", no_argument, 0, 'p'},
        {"cyclic_mode", no_argument, 0, 'c'},
        {"interface", required_argument, 0, 'i'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hpci:", long_options, &option_index);
    if (c == -1) break;
    //printf("c = %d \n", c);
    switch (c) {
        case 'h':
            fprintf(stderr, "Usage: simple_test [options]\n");
        fprintf(stderr, "  Available options\n");
        fprintf(stderr, "    -i, --interface     NIC interface name for EtherCAT network\n");
        fprintf(stderr, "    -p, --position_mode Sample program using Position Profile (pp) mode (Default)\n");
        fprintf(stderr, "    -c, --cycliec_mode  Sample program using cyclic synchronous position(csp) mode\n");
        fprintf(stderr, "    -h, --help          Print this message and exit\n");
        exit(0);
        break;
    case 'p':
        operation_mode = 0x01; // (pp) position profile mode p82
        break;
    case 'c':
        operation_mode = 0x08; // (csp) cyclic synchronous position mode p82
        break;
    case 'i':
        ifname = optarg;
        break;
    }
  }
	*/
  /* start slaveinfo */
  ethercat::EtherCatManager manager(ifname);
  std::vector<minas_control::MinasClient *> clients;
    for (int i = 0; i < manager.getNumClients(); i++ )
    {
      clients.push_back(new minas_control::MinasClient(manager, i+1));
    }

  for (std::vector<minas_control::MinasClient*>::iterator it = clients.begin(); it != clients.end(); ++it)
    {
      minas_control::MinasClient* client = (*it);
      // clear error
      client->reset();

      // set paramete from PANATERM test program
      client->setTrqueForEmergencyStop(100); // 100%
      client->setOverLoadLevel(50);          // 50%
      client->setOverSpeedLevel(120);        // r/min
      client->setMotorWorkingRange(0.1);     // 0.1

      client->setInterpolationTimePeriod(4000);     // 4 msec

      // servo on
      client->servoOn();

      // get current positoin
      minas_control::MinasInput input = client->readInputs();
      int32 current_position = input.position_actual_value;

      // set target position
      minas_control::MinasOutput output;
      // set memory
      memset(&output, 0x00, sizeof(minas_control::MinasOutput));
    if ( operation_mode == 0x01 )
	{ // (pp) position profile mode
        output.target_position = (current_position > 0)?(current_position - 0x100000):(current_position + 0x100000);
	}
    else
	{ // (csp) cyclic synchronous position mode
        output.target_position = current_position;
	}
        output.max_motor_speed = 120;  // rad/min
        output.target_torque = 500;    // 0% (unit 0.1%)
        output.max_torque    = 500;    // 50% (unit 0.1%)
        output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5) p77

      // change to cyclic synchronous position mode
        output.operation_mode = operation_mode;
      //output.operation_mode = 0x08; // (csp) cyclic synchronous position mode
      //output.operation_mode = 0x01; // (pp) position profile mode

      // set profile velocity
        client->setProfileVelocity(0x20000000);

      // pp control model setup (see statusword(6041.h) 3) p.79)
        client->writeOutputs(output);
      // operation mode specific
        while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)  p79
            input = client->readInputs();
        }
        output.controlword   &= ~0x0010; // clear new-set-point (bit4)
        client->writeOutputs(output);

        printf("target position = %08x\n", output.target_position);
    }

  // 4ms in nanoseconds
  double period = 4e+6;
  // get curren ttime
  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  timespecInc(tick, period);

  for (int i = 0; i <= 2000; i++ ) {
    for (std::vector<minas_control::MinasClient*>::iterator it = clients.begin(); it != clients.end(); ++it)
      {
	minas_control::MinasClient* client = (*it);
	minas_control::MinasInput input = client->readInputs();
	minas_control::MinasOutput output = client->readOutputs();
	if ( i % 10 == 0)
	  {
	    printf("err = %04x, ctrl %04x, status %04x, op_mode = %2d, pos = %08x, vel = %08x, tor = %08x\n",
		   input.error_code, output.controlword, input.statusword, input.operation_mode, input.position_actual_value, input.velocity_actual_value, input.torque_actual_value);
	    if ( input.statusword & 0x0400 ) { // target reached (bit 10)
	      printf("target reached\n");
	      break;
	    }
	    printf("Tick %8d.%09d\n", tick.tv_sec, tick.tv_nsec);
	    printf("Input:\n");
	    printf(" 603Fh %08x :Error code\n", input.error_code);
	    printf(" 6041h %08x :Statusword\n", input.statusword);
	    printf(" 6061h %08x :Modes of operation display\n", input.operation_mode);
	    printf(" 6064h %08x :Position actual value\n", input.position_actual_value);
	    printf(" 606Ch %08x :Velocity actual value\n", input.velocity_actual_value);
	    printf(" 6077h %08x :Torque actual value\n", input.torque_actual_value);
	    printf(" 60B9h %08x :Touch probe status\n", input.touch_probe_status);
	    printf(" 60BAh %08x :Touch probe pos1 pos value\n", input.touch_probe_posl_pos_value);
	    printf(" 60FDh %08x :Digital inputs\n", input.digital_inputs);
	    printf("Output:\n");
	    printf(" 6040h %08x :Controlword\n", output.controlword);
	    printf(" 6060h %08x :Mode of operation\n", output.operation_mode);
	    printf(" 6071h %08x :Target Torque\n", output.target_torque);
	    printf(" 6072h %08x :Max Torque\n", output.max_torque);
	    printf(" 607Ah %08x :Target Position\n", output.target_position);
	    printf(" 6080h %08x :Max motor speed\n", output.max_motor_speed);
	    printf(" 60B8h %08x :Touch Probe function\n", output.touch_probe_function);
	    printf(" 60FFh %08x :Target Velocity\n", output.target_velocity);
	    printf(" 60B0h %08x :Position Offset\n", output.position_offset);
	  }

	//output.controlword   |= 0x0004; // enable new-set-point (bit4)
	// (csp) cyclic synchronous position mode
	if ( output.operation_mode == 0x08 )
	{
	  output.position_offset = 0x80000*sin(i/200.0);
	}
	client->writeOutputs(output);
      } // for clients

    //usleep(4*1000);
    timespecInc(tick, period);
    // check overrun
    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    double overrun_time = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -  (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
    if (overrun_time > 0.0)
      {
	fprintf(stderr, "  overrun: %f", overrun_time);
      }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
  }

  for (std::vector<minas_control::MinasClient *>::iterator it = clients.begin(); it != clients.end(); ++it)
    {
      minas_control::MinasClient* client = (*it);
      minas_control::MinasInput input = client->readInputs();
      client->printPDSStatus(input);
      client->printPDSOperation(input);
      client->servoOff();
    }

  printf("End program\n");

  return 0;
}

