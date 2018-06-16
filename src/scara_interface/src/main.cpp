/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Modified 2016, by Shadow Robot Company Ltd.
 *  Modified 2017, by Tokyo Opensource Robotics Kyokai Association
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <algorithm>
#include <cstdio>
#include <cstdarg>
#include <getopt.h>
#include <execinfo.h>
#include <csignal>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <numeric>

#include <std_msgs/Float64.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
//#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <string>
#include <vector>
#include <tinyxml.h>
//
#include <minas_control/minas_hardware_interface.h>

using boost::accumulators::accumulator_set;
using boost::accumulators::stats;
using boost::accumulators::extract_result;
using boost::accumulators::tag::max;
using boost::accumulators::tag::mean;
using std::string;
using std::vector;
using std::accumulate;
using realtime_tools::RealtimePublisher;

static struct
{
  char *program_;
  bool stats_;
  double period;
  std::string iface_;
  bool simulation_;
}
g_options;

void Usage(const string &msg = "")
{
  fprintf(stderr, "Usage: %s [options]\n", g_options.program_);
  fprintf(stderr, "  Available options\n");
  fprintf(stderr, "    -i, --interface             NIC interface name for EtherCAT\n");
  fprintf(stderr, "    -l, --loopback              Use loopback interface for Controller (i.e. simulation mode)\n");
  fprintf(stderr, "    -p, --period                RT loop period in msec\n");
  fprintf(stderr, "    -s, --stats                 Publish statistics on the RT loop "
      "jitter on \"node_name/realtime\" in seconds\n");
  fprintf(stderr, "    -h, --help                  Print this message and exit\n");
  if (msg != "")
  {
    fprintf(stderr, "Error: %s\n", msg.c_str());
    exit(-1);
  }
  else
    exit(0);
}

static int g_quit = 0;
static const int SEC_2_NSEC = 1e+9;
static const int SEC_2_USEC = 1e6;

static struct
{
  accumulator_set<double, stats<max, mean> > ec_acc;
  accumulator_set<double, stats<max, mean> > cm_acc;
  accumulator_set<double, stats<max, mean> > loop_acc;
  accumulator_set<double, stats<max, mean> > jitter_acc;
  int overruns;
  int recent_overruns;
  int last_overrun;
  int last_severe_overrun;
  double overrun_loop_sec;
  double overrun_ec;
  double overrun_cm;

  // These values are set when realtime loop does not meet performance expectations
  bool rt_loop_not_making_timing;
  double halt_rt_loop_frequency;
  double rt_loop_frequency;
}
g_stats;

static void publishDiagnostics(RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher)
{
  if (publisher.trylock())
  {
    accumulator_set<double, stats<max, mean> > zero;
    vector<diagnostic_msgs::DiagnosticStatus> statuses;
    diagnostic_updater::DiagnosticStatusWrapper status;

    static double max_ec = 0, max_cm = 0, max_loop = 0, max_jitter = 0;
    double avg_ec, avg_cm, avg_loop, avg_jitter;

    avg_ec = extract_result<mean>(g_stats.ec_acc);
    avg_cm = extract_result<mean>(g_stats.cm_acc);
    avg_loop = extract_result<mean>(g_stats.loop_acc);
    max_ec = std::max(max_ec, extract_result<max>(g_stats.ec_acc));
    max_cm = std::max(max_cm, extract_result<max>(g_stats.cm_acc));
    max_loop = std::max(max_loop, extract_result<max>(g_stats.loop_acc));
    g_stats.ec_acc = zero;
    g_stats.cm_acc = zero;
    g_stats.loop_acc = zero;

    // Publish average loop jitter
    avg_jitter = extract_result<mean>(g_stats.jitter_acc);
    max_jitter = std::max(max_jitter, extract_result<max>(g_stats.jitter_acc));
    g_stats.jitter_acc = zero;

    status.addf("Max EtherCAT roundtrip (us)", "%.2f", max_ec * SEC_2_USEC);
    status.addf("Avg EtherCAT roundtrip (us)", "%.2f", avg_ec * SEC_2_USEC);
    status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm * SEC_2_USEC);
    status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm * SEC_2_USEC);
    status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop * SEC_2_USEC);
    status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop * SEC_2_USEC);
    status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * SEC_2_USEC);
    status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * SEC_2_USEC);
    status.addf("Control Loop Overruns", "%d", g_stats.overruns);
    status.addf("Recent Control Loop Overruns", "%d", g_stats.recent_overruns);
    status.addf("Last Control Loop Overrun Cause", "ec: %.2fus, cm: %.2fus",
                g_stats.overrun_ec*SEC_2_USEC, g_stats.overrun_cm * SEC_2_USEC);
    status.addf("Last Overrun Loop Time (us)", "%.2f", g_stats.overrun_loop_sec * SEC_2_USEC);
    status.addf("Realtime Loop Frequency", "%.4f", g_stats.rt_loop_frequency);

    status.name = "Realtime Control Loop";
    if (g_stats.overruns > 0 && g_stats.last_overrun < 30)
    {
      if (g_stats.last_severe_overrun < 30)
        status.level = 1;
      else
        status.level = 0;
      status.message = "Realtime loop used too much time in the last 30 seconds.";
    }
    else
    {
      status.level = 0;
      status.message = "OK";
    }
    g_stats.recent_overruns = 0;
    ++g_stats.last_overrun;
    ++g_stats.last_severe_overrun;

    if (g_stats.rt_loop_not_making_timing)
      status.mergeSummaryf(status.ERROR, "realtime loop only ran at %.4f Hz", g_stats.halt_rt_loop_frequency);

    statuses.push_back(status);
    publisher.msg_.status = statuses;
    publisher.msg_.header.stamp = ros::Time::now();
    publisher.unlockAndPublish();
  }
}

static inline double now()
{
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return static_cast<double>(n.tv_nsec) / SEC_2_NSEC + n.tv_sec;
}

static void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

class RTLoopHistory
{
public:
  RTLoopHistory(unsigned length, double default_value) :
    index_(0),
    length_(length),
    history_(length, default_value)
  {
  }

  void sample(double value)
  {
    index_ = (index_ + 1) % length_;
    history_[index_] = value;
  }

  double average() const
  {
    return accumulate(history_.begin(), history_.end(), 0.0) / static_cast<double>(length_);
  }

protected:
  unsigned index_;
  unsigned length_;
  vector<double> history_;
};

static void* terminate_control(RealtimePublisher<diagnostic_msgs::DiagnosticArray> *publisher,
                               RealtimePublisher<std_msgs::Float64> *rtpublisher,
                               const char* message,
                               const char* data = NULL)
{
  ROS_FATAL(message, data);
  publisher->stop();
  delete rtpublisher;
  ros::shutdown();
  return reinterpret_cast<void*>(- 1);
}

void *controlLoop(void */*unused_param*/)  // NOLINT(readability/casting)
{
  double last_published, last_loop_start;
  int policy;
  TiXmlElement *root;
  TiXmlElement *root_element;

  ros::NodeHandle nh;

  RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher(nh, "/diagnostics", 2);
  RealtimePublisher<std_msgs::Float64> *rtpublisher = NULL;

  // Realtime loop should be running at least 3/4 of given frequency
  // or at specified min acceptable frequency
  double period_in_secs = 1e+9 * g_options.period;
  double given_frequency = 1 / period_in_secs;
  double min_acceptable_rt_loop_frequency = 0.75 * given_frequency;
  if (nh.getParam("min_acceptable_rt_loop_frequency", min_acceptable_rt_loop_frequency))
    ROS_WARN("min_acceptable_rt_loop_frequency changed to %f", min_acceptable_rt_loop_frequency);

  unsigned rt_cycle_count = 0;
  double last_rt_monitor_time;

  // Calculate realtime loop frequency every 200msec
  double rt_loop_monitor_period = 0.2;
  // Keep history of last 3 calculation intervals.
  RTLoopHistory rt_loop_history(3, 1000.0);

  if (g_options.stats_)
    rtpublisher = new RealtimePublisher<std_msgs::Float64>(nh, "realtime", 2);

  // Load robot description
  TiXmlDocument xml;
  struct stat st;

  // Initialize the hardware interface
  minas_control::MinasHardwareInterface robot(g_options.iface_, g_options.simulation_);

  // Create controller manager
  controller_manager::ControllerManager cm(&robot);

  // Publish one-time before entering real-time to pre-allocate message vectors
  publishDiagnostics(publisher);

  // Set to realtime scheduler for this thread
  struct sched_param thread_param;
  policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);

  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  ros::Duration durp(g_options.period / 1e+9);

  // Snap to the nearest second
  tick.tv_nsec = (tick.tv_nsec / g_options.period + 1) * g_options.period;
  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

  last_published = now();
  last_rt_monitor_time = now();
  last_loop_start = now();
  while (!g_quit)
  {
    // Track how long the actual loop takes
    double this_loop_start = now();
    g_stats.loop_acc(this_loop_start - last_loop_start);
    last_loop_start = this_loop_start;

    double start = now();

    ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
    robot.read(this_moment, durp);
    double after_ec = now();
    cm.update(this_moment, durp);
    robot.write(this_moment, durp);
    double end = now();

    g_stats.ec_acc(after_ec - start);
    g_stats.cm_acc(end - after_ec);

    if ((end - last_published) > 1.0)
    {
      publishDiagnostics(publisher);
      last_published = end;
    }

    // Realtime loop should run about with the set frequency by default 1000Hz.
    // Missing timing on control cycles usually causes a controller glitch and actuators to jerk.
    // When realtime loop misses a lot of cycles controllers will perform poorly and may cause robot to shake.
    ++rt_cycle_count;
    if ((start - last_rt_monitor_time) > rt_loop_monitor_period)
    {
      // Calculate new average rt loop frequency
      double rt_loop_frequency = static_cast<double>(rt_cycle_count) / rt_loop_monitor_period;

      // Use last X samples of frequency when deciding whether or not to halt
      rt_loop_history.sample(rt_loop_frequency);
      double avg_rt_loop_frequency = rt_loop_history.average();
      if (avg_rt_loop_frequency < min_acceptable_rt_loop_frequency)
      {
        if (!g_stats.rt_loop_not_making_timing)
        {
          // Only update this value if motors when this first occurs (used for diagnostics error message)
          g_stats.halt_rt_loop_frequency = avg_rt_loop_frequency;
        }
        g_stats.rt_loop_not_making_timing = true;
      }
      g_stats.rt_loop_frequency = avg_rt_loop_frequency;
      rt_cycle_count = 0;
      last_rt_monitor_time = start;
    }

    // Compute end of next g_options.period
    timespecInc(tick, g_options.period);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + static_cast<double>(before.tv_nsec) / SEC_2_NSEC) >
        (tick.tv_sec + static_cast<double>(tick.tv_nsec) / SEC_2_NSEC))
    {
      // Total amount of time the loop took to run
      g_stats.overrun_loop_sec = (before.tv_sec + static_cast<double>(before.tv_nsec) / SEC_2_NSEC) -
        (tick.tv_sec + static_cast<double>(tick.tv_nsec) / SEC_2_NSEC);

      // We overran, snap to next "g_options.period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / g_options.period) * g_options.period;
      timespecInc(tick, g_options.period);

      // initialize overruns
      if (g_stats.overruns == 0)
      {
        g_stats.last_overrun = 1000;
        g_stats.last_severe_overrun = 1000;
      }
      // check for overruns
      if (g_stats.recent_overruns > 10)
        g_stats.last_severe_overrun = 0;
      g_stats.last_overrun = 0;

      ++g_stats.overruns;
      ++g_stats.recent_overruns;
      g_stats.overrun_ec = after_ec - start;
      g_stats.overrun_cm = end - after_ec;
    }

    // Sleep until end of g_options.period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

    // Calculate RT loop jitter
    struct timespec after;
    clock_gettime(CLOCK_REALTIME, &after);
    double jitter = (after.tv_sec - tick.tv_sec + static_cast<double>(after.tv_nsec - tick.tv_nsec) / SEC_2_NSEC);

    g_stats.jitter_acc(jitter);

    // Publish realtime loops statistics, if requested
    if (rtpublisher && rtpublisher->trylock())
    {
      rtpublisher->msg_.data = jitter;
      rtpublisher->unlockAndPublish();
    }
  }

  publisher.stop();
  delete rtpublisher;
  robot.shutdown();
  ros::shutdown();
  return NULL;
}

void quitRequested(int sig)
{
  g_quit = 1;
}


#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;

int main(int argc, char *argv[])
{
  bool mlock_all = true;
  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
  {
    perror("Failed to lock memory. It is recommended to set permission to executables, for example: sudo setcap cap_net_raw,cap_ipc_lock=+ep main");
    mlock_all = false;
  }

  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv, "realtime_loop");

  // Parse options
  g_options.program_ = argv[0];
  g_options.period = 1e+6;  // 1 ms in nanoseconds
  g_options.simulation_ = false; // default use NIC interface

  while (true)
  {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"interface", required_argument, 0, 'i'},
      {"loopback", no_argument, 0, 'l'},
      {"stats", no_argument, 0, 's'},
      {"period", required_argument, 0, 'p'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hi:slp:", long_options, &option_index);
    if (c == -1)
      break;

    switch (c)
    {
      case 'h':
        Usage();
        break;
      case 'i':
        g_options.iface_ = std::string(optarg);
        break;
      case 's':
        g_options.stats_ = true;
        break;
      case 'l':
        g_options.simulation_ = true;
        break;
      case 'p':
        // convert period given in msec to nsec
        g_options.period = fabs(atof(optarg))*1e+6;
        break;
    }
  }
  if ( mlock_all == false ) {
    // when mlock is false, on simulation, it'ok
    if ( g_options.simulation_ == true) {
      ROS_WARN("Continue running without mlockall");
    }
    // on real robot, something wrong, forget to setcap cap_net_raw,cap_ipc_lock=+ep
    if ( g_options.simulation_ == false ) {
      perror("Exitting .. real robots needs mlockall to run in realtime");
      exit(EXIT_FAILURE);
    }
  }
  if (optind < argc)
    Usage("Extra arguments");

  ros::NodeHandle node;

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Start thread
  int rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0);
  if (rv != 0)
  {
    ROS_FATAL("Unable to create control thread: rv = %d", rv);
    exit(EXIT_FAILURE);
  }

  ros::spin();
  pthread_join(controlThread, reinterpret_cast<void **>(&rv));

  return rv;
}

