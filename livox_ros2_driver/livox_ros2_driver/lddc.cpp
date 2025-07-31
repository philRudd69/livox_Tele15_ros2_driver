//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"

#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "livox_interfaces/msg/custom_point.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"

#include "lds_lidar.h"
#include "lds_lvx.h"

namespace livox_ros {

/** Lidar Data Distribute Control--------------------------------------------*/
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      frame_num_(0) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
#if 0
  bag_ = nullptr;
#endif
}

Lddc::~Lddc() {
  if (lds_) {
    lds_->PrepareExit();
  }
}

int32_t Lddc::GetPublishStartTime(LidarDevice *lidar, LidarDataQueue *queue,
                                  uint64_t *start_time,
                                  StoragePacket *storage_packet) {
  /***
   * this function seems to retreive a start_time stamp from the packets in the queue.
   * But if there has been some packet-loss (see else-term) then the queue is emptied compeletely.
   * The packet loss is detected by the timestamp in the packet, the tree cases seem to be:
   * 1. packet came a little later (up to 1/4 of the period) than the full period boundary. use the lower period boundary as start_time. return 0 (false)
   * 2. packet came a little too early, before the full period boundary. use the actual packet time-stamp as start-time. return 0 (false)
   * 3. packet in queue is way too late, empty the queue without processing any of the packets. return -1 (true)
   */
  QueuePrePop(queue, storage_packet);
  uint64_t timestamp =
      GetStoragePacketTimestamp(storage_packet, lidar->data_src);
  uint32_t remaining_time = timestamp % publish_period_ns_;  // abgelaufene zeit // publish_period_ns_ = 1*10^9ns / 10Hz = 1*10^8ns
  uint32_t diff_time = publish_period_ns_ - remaining_time;  // noch verfügbare zeit
  /** Get start time, down to the period boundary */
  if (diff_time > (publish_period_ns_ / 4)) {     // erste 75 millisekunden
    // RCLCPP_INFO(cur_node_->get_logger(), "0 : %u", diff_time);
    *start_time = timestamp - remaining_time;
    return 0;
  } else if (diff_time <= lidar->packet_interval_max) { // ca. halbe millisekunde 200µs für Paket. ca 400µs für paket_interval_max 
    *start_time = timestamp;
    return 0;
  } else {  /* hier sind wir im letzten viertel der frame-perioden zeit als 25ms */
    /** Skip some packets up to the period boundary*/
    // RCLCPP_INFO(cur_node_->get_logger(), "2 : %u", diff_time);
    do {
      if (QueueIsEmpty(queue)) {
        break;
      }
      QueuePopUpdate(queue); /* skip packet */
      RCLCPP_INFO(cur_node_->get_logger(), "Frame %i: Skipping packet in GetPublishStartTime() routine.", frame_num_);
      QueuePrePop(queue, storage_packet); /* what if the queue is empty here??? access-violation buffer overrun oder est stehen falsche daten drin. */
      uint32_t last_remaning_time = remaining_time;
      timestamp = GetStoragePacketTimestamp(storage_packet, lidar->data_src);
      remaining_time = timestamp % publish_period_ns_;
      /** Flip to another period */
      if (last_remaning_time > remaining_time) { /* wir sind schon im nächsten Frame, denn die neue abgelaufene zeit ist jetzt kleiner als die alte abgelaufene zeit */
        RCLCPP_INFO(cur_node_->get_logger(), "Frame %i: Flip to next Frame, exit.", frame_num_);
        break;
      }
      diff_time = publish_period_ns_ - remaining_time;
    } while (diff_time > lidar->packet_interval); /* queue wird geleert bis ... ACHTUNG, hier ist die Bedingung leicht anders als in Zeile 86*/

    /* the remaning packets in queue maybe not enough after skip */
    return -1;  // return true
  }
}

void Lddc::InitPointcloud2MsgHeaderXyzrtl(sensor_msgs::msg::PointCloud2& cloud) {
  /* TODO: change intesity to reflectivity? */ 
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(6);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.point_step = sizeof(LivoxPointXyzrtl);
}

void Lddc::InitPointcloud2MsgHeaderXyzttprrtl(sensor_msgs::msg::PointCloud2& cloud) {
  /* the new point type that contains cartesian and spherical coordinates */ 
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(10);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "time_offset";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "theta";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[5].offset = 20;
  cloud.fields[5].name = "phi";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[6].offset = 24;
  cloud.fields[6].name = "r";
  cloud.fields[6].count = 1;
  cloud.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[7].offset = 28;
  cloud.fields[7].name = "reflectivity";
  cloud.fields[7].count = 1;
  cloud.fields[7].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[8].offset = 32;
  cloud.fields[8].name = "tag";
  cloud.fields[8].count = 1;
  cloud.fields[8].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[9].offset = 33;
  cloud.fields[9].name = "line";
  cloud.fields[9].count = 1;
  cloud.fields[9].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.point_step = sizeof(LivoxPointXyzttprrtl);
}

/* for Livox pointcloud2 with XYZRTL (i.e. purely cartesian) points */
uint32_t Lddc::PublishPointcloud2Xyzrtl(LidarDataQueue *queue, uint32_t packet_num,
                                        uint8_t handle) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  InitPointcloud2MsgHeaderXyzrtl(cloud);
  cloud.data.resize(packet_num * kMaxPointPerEthPacket *
                    sizeof(LivoxPointXyzrtl));
  cloud.point_step = sizeof(LivoxPointXyzrtl);

  uint8_t *point_base = cloud.data.data();
  uint8_t data_source = lidar->data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
  uint32_t is_zero_packet = 0;
  while ((published_packet < packet_num) && !QueueIsEmpty(queue)) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if ((packet_gap > lidar->packet_interval_max) &&
        lidar->data_is_published) {
      // RCLCPP_INFO(cur_node_->get_logger(), "Lidar[%d] packet time interval is %ldns", handle, packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + lidar->packet_interval;
        RCLCPP_INFO(cur_node_->get_logger(), "Frame %i: Time difference of packet no. %i too large to previous packet. Generating dummy packet with points at (0,0,0).", frame_num_, published_packet);
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
      }
    }
    /** Use the first packet timestamp as pointcloud2 msg timestamp */
    if (!published_packet) {
      cloud.header.stamp = rclcpp::Time(timestamp);
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(lidar->raw_data_type);
      if (pf_point_convert) {
        point_base = pf_point_convert(point_base, raw_packet,
            lidar->extrinsic_parameter, line_num);
      } else {
        /** Skip the packet */
        RCLCPP_INFO(cur_node_->get_logger(), "Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      point_base = LivoxPointToPxyzrtl(point_base, raw_packet,
          lidar->extrinsic_parameter, line_num);
    }

    if (!is_zero_packet) {
      QueuePopUpdate(queue);
    } else {
      is_zero_packet = 0;
    }
    cloud.width += single_point_num;
    ++published_packet;
    last_timestamp = timestamp;
  }
  cloud.row_step     = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense     = true;
  cloud.data.resize(cloud.row_step); /** Adjust to the real size */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher
      <sensor_msgs::msg::PointCloud2>>(GetCurrentPublisher(handle));
  if (kOutputToRos == output_type_) {
    publisher->publish(cloud);
    ++frame_num_;
  } else {
#if 0    
    if (bag_) {
      bag_->write(p_publisher->getTopic(), rclcpp::Time(timestamp),
                  cloud);
      ++frame_num_;
    }
#endif    
  }
  if (!lidar->data_is_published) {
    lidar->data_is_published = true;
  }
  return published_packet;
}

/* for Livox pointcloud2 with XYZTTPRRTL (i.e. cartesian +  spherical) points */
uint32_t Lddc::PublishPointCloud2Xyzttprrtl(LidarDataQueue *queue, uint32_t packet_num,
                                            uint8_t handle) {
/***
 * queue: queue of udp packets coming from the sensor
 * packet_num: number of packets that ideally form one pointcloud frame. 
 *             "ideally", because packets can get lost or are skipped by the following algorithm.
 *             E.g. 500 for Tele-15 sensor in dual return mode with spherical coordinates (i.e. data_type = 5).
 * handle: integer identifying the lidar
 */
  uint64_t timestamp = 0;
  uint64_t first_timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;
  // RCLCPP_INFO(cur_node_->get_logger(), "packet_num = %i", packet_num);
  // RCLCPP_INFO(cur_node_->get_logger(), "Frame No. %i", frame_num_);

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue migth be not enough after skip */
    return 0;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  InitPointcloud2MsgHeaderXyzttprrtl(cloud);
  cloud.data.resize(packet_num * kMaxPointPerEthPacket *
                    sizeof(LivoxPointXyzttprrtl));
  cloud.point_step = sizeof(LivoxPointXyzttprrtl);

  uint8_t *point_base = cloud.data.data();
  uint8_t data_source = lidar->data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
  uint32_t point_interval = GetPointInterval(lidar->info.type);
  uint32_t packet_offset_time = 0;
  uint32_t is_zero_packet = 0;
  while ((published_packet < packet_num) && !QueueIsEmpty(queue)) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if ((packet_gap > lidar->packet_interval_max) &&  /* time difference to previous packet too large. Assume UDP packet(s) was lost. */
        lidar->data_is_published) {
      // RCLCPP_INFO(cur_node_->get_logger(), "Lidar[%d] packet time interval is %ldns", handle, packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + lidar->packet_interval;
        RCLCPP_INFO(cur_node_->get_logger(), "Frame %i: Time difference of packet no. %i too large to previous packet. Generating dummy packet with points at (0,0,0).", frame_num_, published_packet);
        ZeroPointDataOfStoragePacket(&storage_packet);  /* generate dummy-points at (0,0,0) */
        is_zero_packet = 1; /* 1 = true */
      }
    }
    /** Use the first packet timestamp as pointcloud2 msg timestamp */
    if (!published_packet) {
      cloud.header.stamp = rclcpp::Time(timestamp);
      packet_offset_time = 0;
      first_timestamp = timestamp;
    } else {
      packet_offset_time = (uint32_t)(timestamp - first_timestamp);
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointTimeConvertHandler pf_point_convert =
          GetTimeConvertHandler(lidar->raw_data_type);
      if (pf_point_convert) {
        point_base = pf_point_convert(point_base, raw_packet,
            lidar->extrinsic_parameter, line_num, packet_offset_time, 
            point_interval);
      } else {
        /** Skip the packet */
        RCLCPP_INFO(cur_node_->get_logger(), "Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      point_base = LivoxPointToPxyzrtl(point_base, raw_packet,
          lidar->extrinsic_parameter, line_num);
    }

    if (!is_zero_packet) { /* wenn ein Zero-Paket in die Point cloud geschrieben wird, wird die Queue nicht gepoppt, stattdessen in der nächsten Loop-iteration nochmal prozessiert. */
      QueuePopUpdate(queue);
    } else {
      is_zero_packet = 0; /* 0 = false */
    }
    cloud.width += single_point_num;
    ++published_packet;
    last_timestamp = timestamp;
  }
  cloud.row_step     = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense     = true;
  cloud.data.resize(cloud.row_step); /** Adjust to the real size */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher
      <sensor_msgs::msg::PointCloud2>>(GetCurrentPublisher(handle));
  if (kOutputToRos == output_type_) {
    publisher->publish(cloud);
    ++frame_num_;
  } else {
#if 0    
    if (bag_) {
      bag_->write(p_publisher->getTopic(), rclcpp::Time(timestamp),
                  cloud);
      ++frame_num_;
    }
#endif    
  }
  if (!lidar->data_is_published) {
    lidar->data_is_published = true;
  }
  return published_packet;
}

void Lddc::FillPointsToPclMsg(PointCloud& pcl_msg, \
    LivoxPointXyzrtl* src_point, uint32_t num) {
  LivoxPointXyzrtl* point_xyzrtl = (LivoxPointXyzrtl*)src_point;
  for (uint32_t i = 0; i < num; i++) {
    pcl::PointXYZI point;
    point.x = point_xyzrtl->x;
    point.y = point_xyzrtl->y;
    point.z = point_xyzrtl->z;
    point.intensity = point_xyzrtl->reflectivity;
    ++point_xyzrtl;
    pcl_msg.points.push_back(point);
  }
}

/* for pcl::pxyzi */
uint32_t Lddc::PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                     uint8_t handle) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  PointCloud cloud;
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;

  uint8_t point_buf[2048];
  uint32_t is_zero_packet = 0;
  uint8_t data_source = lidar->data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
  while ((published_packet < packet_num) && !QueueIsEmpty(queue)) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if ((packet_gap > lidar->packet_interval_max) &&
        lidar->data_is_published) {
      //RCLCPP_INFO(cur_node_->get_logger(), "Lidar[%d] packet time interval is %ldns", handle, packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + lidar->packet_interval;
        RCLCPP_INFO(cur_node_->get_logger(), "Frame %i: Time difference of packet no. %i too large to previous packet. Generating dummy packet with points at (0,0,0).", frame_num_, published_packet);
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
      }
    }
    if (!published_packet) {
      cloud.header.stamp = timestamp / 1000.0;  // to pcl ros time stamp
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(lidar->raw_data_type);
      if (pf_point_convert) {
        pf_point_convert(point_buf, raw_packet, lidar->extrinsic_parameter, \
            line_num);
      } else {
        /* Skip the packet */
        RCLCPP_INFO(cur_node_->get_logger(), "Lidar[%d] unkown packet type[%d]", handle,
                 raw_packet->data_type);
        break;
      }
    } else {
      LivoxPointToPxyzrtl(point_buf, raw_packet, lidar->extrinsic_parameter, \
          line_num);
    }
    LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
    FillPointsToPclMsg(cloud, dst_point, single_point_num);
    if (!is_zero_packet) {
      QueuePopUpdate(queue);
    } else {
      is_zero_packet = 0;
    }
    cloud.width += single_point_num;
    ++published_packet;
    last_timestamp = timestamp;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher
      <sensor_msgs::msg::PointCloud2>>(GetCurrentPublisher(handle));
  if (kOutputToRos == output_type_) {
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud,cloud_ros);
    publisher->publish(cloud_ros);
    ++frame_num_;
  } else {
#if 0    
    if (bag_) {
      bag_->write(p_publisher->getTopic(), rclcpp::Time(timestamp),
                  cloud);
      ++frame_num_;
    }
#endif    
  }
  if (!lidar->data_is_published) {
    lidar->data_is_published = true;
  }
  return published_packet;
}

void Lddc::FillPointsToCustomMsg(livox_interfaces::msg::CustomMsg& livox_msg, \
    LivoxPointXyzrtl* src_point, uint32_t num, uint32_t offset_time, \
    uint32_t point_interval, uint32_t echo_num) {
  LivoxPointXyzrtl* point_xyzrtl = (LivoxPointXyzrtl*)src_point;
  for (uint32_t i = 0; i < num; i++) {
    livox_interfaces::msg::CustomPoint point;
    if (echo_num > 1) { /** dual return mode */
      point.offset_time = offset_time + (i / echo_num) * point_interval;
    } else {
      point.offset_time = offset_time + i * point_interval;
    }
    point.x = point_xyzrtl->x;
    point.y = point_xyzrtl->y;
    point.z = point_xyzrtl->z;
    point.reflectivity = point_xyzrtl->reflectivity;
    point.tag = point_xyzrtl->tag;
    point.line = point_xyzrtl->line;
    ++point_xyzrtl;
    livox_msg.points.push_back(point);
  }
}

uint32_t Lddc::PublishCustomPointcloud(LidarDataQueue *queue,
                                       uint32_t packet_num, uint8_t handle) {
  // static uint32_t msg_seq = 0;
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;

  StoragePacket storage_packet;
  LidarDevice *lidar = &lds_->lidars_[handle];
  if (GetPublishStartTime(lidar, queue, &last_timestamp, &storage_packet)) {
    /* the remaning packets in queue maybe not enough after skip */
    return 0;
  }

  livox_interfaces::msg::CustomMsg livox_msg;
  livox_msg.header.frame_id.assign(frame_id_);
  // livox_msg.header.seq = msg_seq;
  // ++msg_seq;
  livox_msg.timebase = 0;
  livox_msg.point_num = 0;
  livox_msg.lidar_id = handle;

  uint8_t point_buf[2048];
  uint8_t data_source = lds_->lidars_[handle].data_src;
  uint32_t line_num = GetLaserLineNumber(lidar->info.type);
  uint32_t echo_num = GetEchoNumPerPoint(lidar->raw_data_type);
  uint32_t point_interval = GetPointInterval(lidar->info.type);
  uint32_t published_packet = 0;
  uint32_t packet_offset_time = 0;  /** uint:ns */
  uint32_t is_zero_packet = 0;
  while (published_packet < packet_num) {
    QueuePrePop(queue, &storage_packet);
    LivoxEthPacket *raw_packet =
        reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
    int64_t packet_gap = timestamp - last_timestamp;
    if ((packet_gap > lidar->packet_interval_max) &&
        lidar->data_is_published) {
      // RCLCPP_INFO(this->get_logger(), "Lidar[%d] packet time interval is %ldns", handle,
      // packet_gap);
      if (kSourceLvxFile != data_source) {
        timestamp = last_timestamp + lidar->packet_interval;
        RCLCPP_INFO(cur_node_->get_logger(), "Frame %i: Time difference of packet no. %i too large to previous packet. Generating dummy packet with points at (0,0,0).", frame_num_, published_packet);
        ZeroPointDataOfStoragePacket(&storage_packet);
        is_zero_packet = 1;
      }
    }
    /** first packet */
    if (!published_packet) {
      livox_msg.timebase = timestamp;
      packet_offset_time = 0;
      /** convert to ros time stamp */
      livox_msg.header.stamp = rclcpp::Time(timestamp);
    } else {
      packet_offset_time = (uint32_t)(timestamp - livox_msg.timebase);
    }
    uint32_t single_point_num = storage_packet.point_num * echo_num;

    if (kSourceLvxFile != data_source) {
      PointConvertHandler pf_point_convert =
          GetConvertHandler(lidar->raw_data_type);
      if (pf_point_convert) {
        pf_point_convert(point_buf, raw_packet, lidar->extrinsic_parameter, \
            line_num);
      } else {
        /* Skip the packet */
        RCLCPP_INFO(cur_node_->get_logger(), "Lidar[%d] unkown packet type[%d]", handle,
                 lidar->raw_data_type);
        break;
      }
    } else {
      LivoxPointToPxyzrtl(point_buf, raw_packet, lidar->extrinsic_parameter, \
          line_num);
    }
    LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
    FillPointsToCustomMsg(livox_msg, dst_point, single_point_num, \
        packet_offset_time, point_interval, echo_num);

    if (!is_zero_packet) {
      QueuePopUpdate(queue);
    } else {
      is_zero_packet = 0;
    }

    livox_msg.point_num += single_point_num;
    last_timestamp = timestamp;
    ++published_packet;
  }

  rclcpp::Publisher<livox_interfaces::msg::CustomMsg>::SharedPtr publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher
      <livox_interfaces::msg::CustomMsg>>(GetCurrentPublisher(handle));  
  if (kOutputToRos == output_type_) {
    publisher->publish(livox_msg);
    ++frame_num_;
  } else {
#if 0    
    if (bag_) {
      bag_->write(p_publisher->getTopic(), rclcpp::Time(timestamp),
          livox_msg);
      ++frame_num_;
    }
#endif    
  }

  if (!lidar->data_is_published) {
    lidar->data_is_published = true;
  }
  return published_packet;
}

uint32_t Lddc::PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle) {
  uint64_t timestamp = 0;
  uint32_t published_packet = 0;

  sensor_msgs::msg::Imu imu_data;
  imu_data.header.frame_id = "livox_frame";

  uint8_t data_source = lds_->lidars_[handle].data_src;
  StoragePacket storage_packet;
  QueuePrePop(queue, &storage_packet);
  LivoxEthPacket *raw_packet =
      reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
  timestamp = GetStoragePacketTimestamp(&storage_packet, data_source);
  if (timestamp) {
    imu_data.header.stamp =
        rclcpp::Time(timestamp);  // to ros time stamp
  }

  uint8_t point_buf[2048];
  LivoxImuDataProcess(point_buf, raw_packet);

  LivoxImuPoint *imu = (LivoxImuPoint *)point_buf;
  imu_data.angular_velocity.x = imu->gyro_x;
  imu_data.angular_velocity.y = imu->gyro_y;
  imu_data.angular_velocity.z = imu->gyro_z;
  imu_data.linear_acceleration.x = imu->acc_x;
  imu_data.linear_acceleration.y = imu->acc_y;
  imu_data.linear_acceleration.z = imu->acc_z;

  QueuePopUpdate(queue);
  ++published_packet;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher
      <sensor_msgs::msg::Imu>>(GetCurrentImuPublisher(handle));
  if (kOutputToRos == output_type_) {
    publisher->publish(imu_data);
  } else {
#if 0    
    if (bag_) {
      bag_->write(p_publisher->getTopic(), rclcpp::Time(timestamp),
                  imu_data);
    }
#endif    
  }
  return published_packet;
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue->storage_packet == nullptr) {
    return;
  }

  while (!QueueIsEmpty(p_queue)) {
    uint32_t used_size = QueueUsedSize(p_queue);
    uint32_t onetime_publish_packets = lidar->onetime_publish_packets;
    if (used_size < onetime_publish_packets) {
      break;
    }

    if (kPointCloud2XyzrtlMsg == transfer_format_) {
      PublishPointcloud2Xyzrtl(p_queue, onetime_publish_packets, handle);
    } else if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(p_queue, onetime_publish_packets, handle);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPointcloudData(p_queue, onetime_publish_packets, handle);
    } else if (kPointCloud2XyzttprrtlMsg == transfer_format_ && lidar->config.coordinate==1){
      PublishPointCloud2Xyzttprrtl(p_queue, onetime_publish_packets, handle);
    } else if (kPointCloud2XyzttprrtlMsg == transfer_format_ && lidar->config.coordinate==0){
      RCLCPP_WARN_THROTTLE(cur_node_->get_logger(), *cur_node_->get_clock(), 1000,
                           "xfer_format = Livox Pointcloud(XYZTTPRRTL) (=4) but coordinate = cartesian (=0)." \
                           "This is not possible. Switching to xfer_format = Livox Pointcloud(XYZRTL) (=0)");
      PublishPointcloud2Xyzrtl(p_queue, onetime_publish_packets, handle);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t handle, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->imu_data;
  if (p_queue->storage_packet == nullptr) {
    return;
  }
  while (!QueueIsEmpty(p_queue)) {
    PublishImuData(p_queue, 1, handle);
  }
}

void Lddc::DistributeLidarData(void) {
  if (lds_ == nullptr) {
    return;
  }
  lds_->semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) ||
        (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);
    PollingLidarImuData(lidar_id, lidar);
  }

  if (lds_->IsRequestExit()) {
    PrepareExit();
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
    std::string &topic_name, uint32_t queue_size) {
    if (kPointCloud2XyzrtlMsg == msg_type || kPointCloud2XyzttprrtlMsg == msg_type) {
      RCLCPP_INFO(cur_node_->get_logger(),
          "%s publish use PointCloud2 format", topic_name.c_str());
      return cur_node_->create_publisher<
          sensor_msgs::msg::PointCloud2>(topic_name, queue_size);
    } else if (kLivoxCustomMsg == msg_type) {
      RCLCPP_INFO(cur_node_->get_logger(),
          "%s publish use livox custom format", topic_name.c_str());
      return cur_node_->create_publisher<
          livox_interfaces::msg::CustomMsg>(topic_name, queue_size);
    }
#if 0
    else if (kPclPxyziMsg == msg_type)  {
      RCLCPP_INFO(cur_node_->get_logger(),
          "%s publish use pcl PointXYZI format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud>(topic_name, queue_size);
    }
#endif    
    else if (kLivoxImuMsg == msg_type)  {
      RCLCPP_INFO(cur_node_->get_logger(),
          "%s publish use imu format", topic_name.c_str());
      return cur_node_->create_publisher<sensor_msgs::msg::Imu>(topic_name,
          queue_size);
    } else {
      std::shared_ptr<rclcpp::PublisherBase>null_publisher(nullptr);
      return null_publisher;
    }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
          lds_->lidars_[handle].info.broadcast_code);
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_pub_[handle] = CreatePublisher(transfer_format_, topic_name,
          queue_size);
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return global_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_imu_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
          lds_->lidars_[handle].info.broadcast_code);
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name,
          queue_size);
    }
    return private_imu_pub_[handle];
  } else {
    if (!global_imu_pub_) {
      std::string topic_name("livox/imu");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return global_imu_pub_;
  }
}

void Lddc::CreateBagFile(const std::string &file_name) {
  // if (!bag_) {
  //   bag_ = new rosbag::Bag;
  //   bag_->open(file_name, rosbag::bagmode::Write);
  //   RCLCPP_INFO(cur_node_->get_logger(), "Create bag file :%s!", file_name.c_str());
  // }
}

void Lddc::PrepareExit(void) {
  // if (bag_) {
  //   RCLCPP_INFO(cur_node_->get_logger(), "Waiting to save the bag file!");
  //   bag_->close();
  //   RCLCPP_INFO(cur_node_->get_logger(), "Save the bag file successfully!");
  //   bag_ = nullptr;
  // }
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

}  // namespace livox_ros
