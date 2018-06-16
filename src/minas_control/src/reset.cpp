/** \file
 * \brief Example code for reset Simple Open EtherCAT master
 *
 * Usage : reset [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 */

#include <stdio.h>
#include <ethercat_manager/ethercat_manager.h>
#include <minas_control/minas_client.h>

int main(int argc, char *argv[])
{
  printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

  if (argc > 1)
    {      
      /* start slaveinfo */
      std::string ifname(argv[1]);
      ethercat::EtherCatManager manager(ifname);
      minas_control::MinasClient client(manager, 1);

      // clear error
      client.reset();
    }
  else
    {
      printf("Usage: reset ifname1\nifname = eth0 for example\n");
    }   
   
  printf("End program\n");

  return 0;
}

