/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/


/* basically an reduced version of Lddc::GetPublishStartTime() */

#include <iostream>
#include <stdint.h>

using namespace std;

// Driver code
int main(void)
{
    uint32_t point_interval = 4167;
  	uint32_t points_per_packet = 48;
    uint32_t packet_interval = point_interval * points_per_packet;
  	uint32_t packet_interval_max = packet_interval * 1.8f;
  	uint64_t start_time;
  	uint64_t timestamp = 1099900000;
  	int64_t kNsPerSecond = 1000000000;
  	double publish_frq_ = 10;
  	uint32_t publish_period_ns_ = kNsPerSecond / publish_frq_;
  	uint32_t remaining_time = timestamp % publish_period_ns_;
  	uint32_t diff_time = publish_period_ns_ - remaining_time;
  	cout << "timestamp of packet: " << timestamp << endl;
  	cout << "packet_interval: " << packet_interval << endl;
  	cout << "remaining_time: " << remaining_time << endl;
  	cout << "diff_time: " << diff_time << endl;
  	cout << "publish_period_ns_/4: " << publish_period_ns_/4 << endl;
  	cout << "packet_interval_max: " << packet_interval_max << endl;
  	/** Get start time, down to the period boundary */
  	if (diff_time > (publish_period_ns_ / 4)) {
    start_time = timestamp - remaining_time;
    cout << "1st case. start_time = " << start_time << endl;
  	} else if (diff_time <= packet_interval_max) {
    start_time = timestamp;
    cout << "2nd case. start_time = " << start_time << endl;
  	} else {
    /** Skip some packets up to the period boundary*/
    do {
      
      cout << "3rd case. skip packet" << endl;
      uint32_t last_remaning_time = remaining_time;
      timestamp = timestamp + packet_interval; /* is this correct ???? */
      remaining_time = timestamp % publish_period_ns_;
      /** Flip to another period */
      if (last_remaning_time > remaining_time) {
        cout << "flip" << endl;
      }
      diff_time = publish_period_ns_ - remaining_time;
    } while (diff_time > packet_interval);
  	}
    return 0;
}
