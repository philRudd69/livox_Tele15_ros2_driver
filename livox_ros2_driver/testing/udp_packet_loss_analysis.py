from pathlib import Path
import os
from rosbags.highlevel import AnyReader
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import plotly.express as px
import plotly.graph_objs as go


class recording:
    def __init__(self, rosbag: str,
                 spherical_points_available: bool,
                 detect_missing_points: bool,
                 detect_time_jumps_between_frames: bool,
                 benchmark_driver: bool,
                 proprietary_driver: bool,
                 show_plots: bool = True,
                 topics_to_analyse=[""]) -> None:
        """
        constructor
        """
        # members related to the AnyReader
        self._rosbag = Path(rosbag)
        self._ros_bag_paths = []
        self._ros_bag_names = []
        self._ros_reader = None
        self._ros_conns = None
        self._ros_topics = None
        self._ros_typestore = None
        self._ros_topics_to_analyse = topics_to_analyse

        # members defining the kind of analysis we want to perform
        self._detect_missing_points = detect_missing_points
        self._detect_time_jumps_between_frames = detect_time_jumps_between_frames
        self._spherical_points_available = spherical_points_available
        self._benchmark_driver = benchmark_driver
        self._proprietary_driver = proprietary_driver

        if self._spherical_points_available and self._benchmark_driver:
            print(f"Options spherical_points_available and benchmark_driver were set True, this is not possible.")
            exit
        if self._spherical_points_available and self._proprietary_driver:
            print(f"Options spherical_points_available and proprietary_driver were set True, this is not possible.")
            exit

        # members for the analysis
        self._previous_last_time_stamp = 0
        self._previous_df = pd.DataFrame()
        self._frames_w_large_gaps_to_prev_frames = []
        self._gapsize_in_pts = []

        # members for plotting
        self._show_plots = show_plots
        self._plotting_dir = None

        # execute processing steps
        self._getFilename()
        self._loadROSbag()  # load the rosbag, get all topic names
        self._plotting_dir = self._ros_bag_paths[0].parents[0]
        self._analyseROSbag()
        self._closeROSbag()  # close the AnyReader

    def _getFilename(self) -> None:
        """
        retrieves the file names from the self._ros_bag_dir user input.
        Note: Necessary, because only for ROS1 files the user will directly give the filename, for ROS2 they will provide the directory only.
        """
        if self._rosbag.exists():
            if self._rosbag.is_dir():  # user input is directory of ROS2 files (.db3 + metadata.yaml)
                self._ros_bag_paths = list(self._rosbag.glob("*.db3")) # find the files
                self._ros_bag_names = [path.stem for path in self._ros_bag_paths]
            elif self._rosbag.is_file() and self._rosbag.suffix == '.bag':  # user input is path of single ROS file (.bag)
                self._ros_bag_paths = [self._rosbag]
                self._ros_bag_names = [self._rosbag.stem]
        else:
            error_msg = f"Specified 'rosbag' path/dir is not existing!"
            print(error_msg)
            raise ValueError(error_msg)

    def _loadROSbag(self) -> None:
        """
        loads the ROS1 or ROS2 bagfile.
        Note: Directory or path of bagfile is given in costructor
        """
        try:
            self._ros_reader = AnyReader([self._rosbag])
            self._ros_reader.open()
            print(f"Opened ROS file {self._rosbag}")
            self._ros_typestore = self._ros_reader.typestore  # handles msg definitions
            # get all connections and all topics from the connections
            self._ros_conns = [conn for conn in self._ros_reader.connections]
            self._ros_topics = self._ros_reader.topics  #[conn.topic for conn in self._ros_conns]
        except Exception as e:
            print(e)
        print(f"Number of connections: {len(self._ros_conns)}")
        print(f"The {len(self._ros_conns)} Topic Names are:")
        for topic in self._ros_topics:
            print(f"- {topic}")

    def _closeROSbag(self) -> None:
        """
        ROS bag file opened with AnyReader must be explicitely closed.
        """
        self._ros_reader.close()

    def _filterConnections(self, connections, topic_names):
        """
        Filters the connections (from rosbags lib) by topic names listed in topic_names list
        """
        filtered_conns = []
        for conn in connections:
            if conn.topic in topic_names:
                # print(f"found {conn.topic} in {topic_names}")
                filtered_conns.append(conn)

        # print(f"Number of filtered connections: {len(filtered_conns)}")
        # for conn in filtered_conns:
            # print(f"- {conn}")
        return filtered_conns

    def _processMsgsInROSbag(self, topic_name, func) -> None:

        # filter the connections for those we are interested in
        filtered_conns = self._filterConnections(self._ros_conns, topic_name)
        msgs = self._ros_reader.messages(connections=filtered_conns)

        for m, msg in enumerate(msgs):
            (connection, timestamp, rawdata) = msg
            if connection.topic == topic_name:
                # deserialize data 
                deserialized_msg = self._ros_reader.deserialize(rawdata, connection.msgtype)
                # optional: print the point field definition (if you have a new point definition)
                # self._printPointFieldStructure(deserialized_msg)
                # call input function with deserialized data
                func(deserialized_msg, m)
                # if m==20:
                #    break

    def _analyseROSbag(self):
        for topic in self._ros_topics_to_analyse:
            if self._detect_missing_points:
                self._processMsgsInROSbag(topic_name=topic, func=self._detectMissingPoints)
            if self._detect_time_jumps_between_frames:
                self._processMsgsInROSbag(topic_name=topic, func=self._detectTimeJumpsBetweenFrames)
                self._plotHistograms()

    def _plotHistograms(self) -> None:
        """
        plots histograms for analysis of
        - how often UDP packet losses appear, like every n'th frame
        - how many points are lost in between frames
        """
        diff = np.diff(self._frames_w_large_gaps_to_prev_frames)
        missing_pts = np.array(self._gapsize_in_pts)
        df = pd.DataFrame(diff, columns=['n_frames'])
        df1 = pd.DataFrame(missing_pts, columns=['missing_pts'])
        ax = df.plot.hist(column=["n_frames"], figsize=(7,7))
        ax.set_title(f"Losing UDP packets between frames occurs every ... frame:")
        ax.set_xlabel("n frames")
        filename = str(self._plotting_dir) + '/Losing UDP packets between frames occurs every ... frame.png'
        plt.savefig(filename)
        print(f"saving to {filename}")
        ax1 = df1.plot.hist(column=["missing_pts"], figsize=(7,7))
        ax1.set_title(f"Amount of points lost due to UDP packets loss between frames")
        ax1.set_xlabel("n points")
        filename = str(self._plotting_dir) + '/Amount of points lost due to UDP packets loss between frames.png'
        plt.savefig(filename)
        print(f"saving to {filename}")

    def _msgToDataFrame(self, msg) -> pd.DataFrame:
        """
        Livox ROS2 message containing cartesian and spherical coordinates (XYZTTPRRTL) to DataFrame.
        """
        if self._spherical_points_available:
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape((-1, msg.point_step))
            x = np.frombuffer(np.ascontiguousarray(data[:, :4]), dtype=np.float32)
            y = np.frombuffer(np.ascontiguousarray(data[:, 4:8]), dtype=np.float32)
            z = np.frombuffer(np.ascontiguousarray(data[:, 8:12]), dtype=np.float32)
            time_offset = np.frombuffer(np.ascontiguousarray(data[:, 12:16]), dtype=np.uint32)
            az_rad = np.frombuffer(np.ascontiguousarray(data[:, 16:20]), dtype=np.float32)
            el_rad = np.frombuffer(np.ascontiguousarray(data[:, 20:24]), dtype=np.float32)
            range_m = np.frombuffer(np.ascontiguousarray(data[:, 24:28]), dtype=np.float32)
            reflectivity = np.frombuffer(np.ascontiguousarray(data[:, 28:32]), dtype=np.float32)
            tag = np.frombuffer(np.ascontiguousarray(data[:, 32:33]), dtype=np.uint8)
            line = np.frombuffer(np.ascontiguousarray(data[:, 33:34]), dtype=np.uint8)
            az_rad = np.where(az_rad > np.pi, az_rad - 2.0 * np.pi, az_rad)
            az_deg = np.rad2deg(az_rad)
            el_deg = np.rad2deg(el_rad)

            new_df = pd.DataFrame({'x': x,
                                   'y': y,
                                   'z': z,
                                   'time_offset': time_offset,
                                   'az_deg': az_deg,
                                   'el_deg': el_deg,
                                   'az_rad': az_rad,
                                   'el_rad': el_rad,
                                   'range_m': range_m,
                                   'reflectivity': reflectivity,
                                   'tag': tag,
                                   'line': line})
            # calculate No-Hit Flag:
            new_df["No-Hit"] = new_df["range_m"] == 0.0
            new_df['time_step'] = new_df['time_offset'].diff()
            return new_df  
        elif self._benchmark_driver:
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape((-1, msg.point_step))
            x = np.frombuffer(np.ascontiguousarray(data[:, :4]), dtype=np.float32)
            y = np.frombuffer(np.ascontiguousarray(data[:, 4:8]), dtype=np.float32)
            z = np.frombuffer(np.ascontiguousarray(data[:, 8:12]), dtype=np.float32)
            reflectivity = np.frombuffer(np.ascontiguousarray(data[:, 12:16]), dtype=np.float32)
            tag = np.frombuffer(np.ascontiguousarray(data[:, 16:17]), dtype=np.uint8)
            line = np.frombuffer(np.ascontiguousarray(data[:, 17:18]), dtype=np.uint8)
            timestamp = np.frombuffer(np.ascontiguousarray(data[:, 18:26]), dtype=np.double)
            timestamp_ns = timestamp * 1000000000.0
            range_m = np.sqrt(x**2+y**2+z**2)
            range_m[np.isnan(range_m)] = 0.0
            z[np.isnan(z)] = 0.0
            range_m[np.isinf(range_m)] = 999.0
            z[np.isinf(z)] = 999.9
            # TODO: somehow the division true_divide() fails sometimes, but not due to NANs or INFs, because we catch them
            argument = np.true_divide(z, range_m)
            el_rad = np.where(range_m > 0.0, np.arcsin(argument), 0.0)
            az_rad = np.where(x != 0.0, np.arctan2(y, x), 0.0)
            az_rad = np.where(az_rad > np.pi, az_rad - 2.0 * np.pi, az_rad)
            az_deg = np.rad2deg(az_rad)
            el_deg = np.rad2deg(el_rad)

            new_df = pd.DataFrame({'x': x,
                                   'y': y,
                                   'z': z,
                                   'az_deg': az_deg,
                                   'el_deg': el_deg,
                                   'az_rad': az_rad,
                                   'el_rad': el_rad,
                                   'range_m': range_m,
                                   'reflectivity': reflectivity,
                                   'tag': tag,
                                   'line': line,
                                   'timestamp': timestamp_ns})
            # calculate No-Hit Flag:
            new_df["No-Hit"] = new_df["range_m"] == 0.0
            return new_df
        elif self._proprietary_driver:  # proprietary driver to inspect
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape((-1, msg.point_step))
            x = np.frombuffer(np.ascontiguousarray(data[:, :4]), dtype=np.float32)
            y = np.frombuffer(np.ascontiguousarray(data[:, 4:8]), dtype=np.float32)
            z = np.frombuffer(np.ascontiguousarray(data[:, 8:12]), dtype=np.float32)
            reflectivity = np.frombuffer(np.ascontiguousarray(data[:, 12:16]), dtype=np.float32)
            line = np.tile(np.array([0, 1, 2, 3, 4, 5]), int(len(x)/6))
            range_m = np.sqrt(x**2+y**2+z**2)
            el_rad = np.where(range_m != 0.0, np.arcsin(z/range_m), 0.0)
            az_rad = np.where(x != 0, np.arctan2(y, x), 0.0)
            az_rad = np.where(az_rad > np.pi, az_rad - 2.0 * np.pi, az_rad)
            az_deg = np.rad2deg(az_rad)
            el_deg = np.rad2deg(el_rad)

            new_df = pd.DataFrame({'x': x,
                                   'y': y,
                                   'z': z,
                                   'az_deg': az_deg,
                                   'el_deg': el_deg,
                                   'az_rad': az_rad,
                                   'el_rad': el_rad,
                                   'range_m': range_m,
                                   'reflectivity': reflectivity,
                                   'line': line})
            # calculate No-Hit Flag:
            new_df["No-Hit"] = new_df["range_m"] == 0.0

            if len(x) % 24000 != 0:
                print(f"Frame coming from proprietary driver is incomplete. point count={len(x)}")
            return new_df
        else:  # original livox driver delivering just xyz
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape((-1, msg.point_step))
            x = np.frombuffer(np.ascontiguousarray(data[:, :4]), dtype=np.float32)
            y = np.frombuffer(np.ascontiguousarray(data[:, 4:8]), dtype=np.float32)
            z = np.frombuffer(np.ascontiguousarray(data[:, 8:12]), dtype=np.float32)
            reflectivity = np.frombuffer(np.ascontiguousarray(data[:, 12:16]), dtype=np.float32)
            tag = np.frombuffer(np.ascontiguousarray(data[:, 16:17]), dtype=np.uint8)
            line = np.frombuffer(np.ascontiguousarray(data[:, 17:18]), dtype=np.uint8)
            range_m = np.sqrt(x**2+y**2+z**2)
            el_rad = np.where(range_m != 0.0, np.arcsin(z/range_m), 0.0)
            az_rad = np.where(x != 0, np.arctan2(y, x), 0.0)
            az_rad = np.where(az_rad > np.pi, az_rad - 2.0 * np.pi, az_rad)
            az_deg = np.rad2deg(az_rad)
            el_deg = np.rad2deg(el_rad)

            new_df = pd.DataFrame({'x': x,
                                   'y': y,
                                   'z': z,
                                   'az_deg': az_deg,
                                   'el_deg': el_deg,
                                   'az_rad': az_rad,
                                   'el_rad': el_rad,
                                   'range_m': range_m,
                                   'reflectivity': reflectivity,
                                   'tag': tag,
                                   'line': line})
            # calculate No-Hit Flag:
            new_df["No-Hit"] = new_df["range_m"] == 0.0
            return new_df

    def _detectTimeJumpsBetweenFrames(self, msg, msg_idx) -> None:
        if self._spherical_points_available:
            df = self._msgToDataFrame(msg)
            if head := getattr(msg, 'header', None):
                first_time_stamp = head.stamp.sec * 10**9 + head.stamp.nanosec + df.iloc[0]['time_offset']
                last_time_stamp = head.stamp.sec * 10**9 + head.stamp.nanosec + df.iloc[-1]['time_offset']
                diff = 0
                if self._previous_last_time_stamp != 0:
                    diff = first_time_stamp - self._previous_last_time_stamp
                    print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
                else:
                    print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
                if len(self._previous_df) > 0:
                    if diff > 4167 * 1.8:
                        self._plotGapBetweenDataFrames(self._previous_df, df, msg_idx)
                        self._frames_w_large_gaps_to_prev_frames.append(msg_idx)
                        self._gapsize_in_pts.append(diff/4167.0)
                else:
                    print(f"{msg_idx} No previous DataFrame/msg available.")
                self._previous_last_time_stamp = last_time_stamp
                self._previous_df = df.copy(deep=True)
            else:
                print(f"{msg_idx} header not found. skipping this msg.")

        elif self._benchmark_driver:
            df = self._msgToDataFrame(msg)
            first_time_stamp = df.iloc[0]['timestamp']
            last_time_stamp = df.iloc[-1]['timestamp']
            diff = 0
            if self._previous_last_time_stamp != 0:
                diff = first_time_stamp - self._previous_last_time_stamp
                print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
            else:
                print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
            if len(self._previous_df) > 0:
                if diff > 4167 * 1.8:
                    self._plotGapBetweenDataFrames(self._previous_df, df, msg_idx)
                    self._frames_w_large_gaps_to_prev_frames.append(msg_idx)
                    self._gapsize_in_pts.append(diff/4167.0)
            else:
                print(f"{msg_idx} No previous DataFrame/msg available.")
            self._previous_last_time_stamp = last_time_stamp
            self._previous_df = df.copy(deep=True)

        elif self._proprietary_driver:
            df = self._msgToDataFrame(msg)
            first_time_stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            last_time_stamp = first_time_stamp + 99995841  # 4166.667ns * 23999 the exact time is hard to guess
            diff = 0
            if self._previous_last_time_stamp != 0:
                diff = first_time_stamp - self._previous_last_time_stamp
                print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
            else:
                print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
            if len(self._previous_df) > 0:
                if diff > 4167 * 48:  # 1.8:  # because there are at least 48pts in one UDP packet
                    self._plotGapBetweenDataFrames(self._previous_df, df, msg_idx)
                    self._frames_w_large_gaps_to_prev_frames.append(msg_idx)
                    self._gapsize_in_pts.append(diff/4167.0)
            else:
                print(f"{msg_idx} No previous DataFrame/msg available.")
            self._previous_last_time_stamp = last_time_stamp
            self._previous_df = df.copy(deep=True)

        elif not self._spherical_points_available \
                and not self._benchmark_driver \
                and not self._proprietary_driver:
            # case where we have original livox driver with xfer_format [0,1,2]
            df = self._msgToDataFrame(msg)
            first_time_stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            last_time_stamp = first_time_stamp + 99995833  # 4166.66ns * 23999 the exact time is hard to guess
            diff = 0
            if self._previous_last_time_stamp != 0:
                diff = first_time_stamp - self._previous_last_time_stamp
                print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
            else:
                print(f"{msg_idx} Time-difference to previous frame: {diff}ns")
            if len(self._previous_df) > 0:
                if diff > 4167 * 1.8:
                    self._plotGapBetweenDataFrames(self._previous_df, df, msg_idx)
                    self._frames_w_large_gaps_to_prev_frames.append(msg_idx)
                    self._gapsize_in_pts.append(diff/4167.0)
            else:
                print(f"{msg_idx} No previous DataFrame/msg available.")
            self._previous_last_time_stamp = last_time_stamp
            self._previous_df = df.copy(deep=True)

        else:
            print("Invalid combination of parameters. Cannot execute _detectTimeJumpsBetweenFrames()")

    def _plotGapBetweenDataFrames(self, df_a, df_b, msg_idx) -> None:
        """
        print the last 120 points of df_a and the first 120 points of df_b
        """
        last_pts = df_a.iloc[-120:-1]
        first_pts = df_b.iloc[:120]
        ################# plot of pattern colored by time_offset #################
        ax = last_pts.plot.scatter(x="az_deg", y="el_deg", c="b", s=1, figsize=(7, 7))  #, name=f"previous {msg_idx-1}")
        ax.set_aspect('equal')
        ax.set_title(f"Livox Tele-15: Gap in Scanning Pattern between Frames {msg_idx-1} and {msg_idx}")
        ax.set_ylabel("Elevation [deg]")
        ax.set_xlabel("Azimuth [deg]")
        #ax.set_ylim(min_el-1, max_el+1)
        #ax.set_xlim(min_az-1, max_az+1)
        first_pts.plot.scatter(x="az_deg", y="el_deg", c="r", s=3, ax=ax)  #, name=f"current {msg_idx}")
        filename = os.path.join(str(self._plotting_dir), f'Livox_Tele-15_pattern_Gap_between_frame_{msg_idx-1}_and_{msg_idx}.png')
        plt.savefig(filename)
        print(f"saving to {filename}")
        if self._show_plots:
            plt.show()
    
    def _printPointFieldStructure(self, msg):
        """
        check the definition of the pointFields array to know how the data is structured
        """
        print("\n   - Content of the PointFields array:")
        print(*msg.fields, sep='\n')
        print(f"type of the msg.data = {type(msg.data)}")
        print(f"Stepsize between points in data according to width info: {len(msg.data)/msg.width}")

    def _detectMissingPoints(self, msg, msg_idx) -> None:
        """
        reads point cloud and processes its contents
        """
        if self._spherical_points_available:
            new_df = self._msgToDataFrame(msg)
            # analyse the weird 576 invalid points without any directions every 10th frame
            invalid_points_df = new_df[new_df['el_deg']==90.0]
            n_invalid_pts = len(invalid_points_df)
            if n_invalid_pts > 0:
                print(f"tag info of the invalid points: \n{invalid_points_df['tag'].value_counts()}")
                # --> seems as if the tag is always equal to 0
                print(f"line numbers of the invalid points: \n{invalid_points_df['line'].value_counts()}")
                # --> seems as if there are equal amounts for every laser/line, that is 96.
                print(f"range_m of the invalid points: \n{invalid_points_df['range_m'].value_counts()}")
                print(f"time_offset of the invalid points: \n{invalid_points_df['time_offset'].describe()}")
                # inspecting the time_offset a little bit further:
                max_time_offset = invalid_points_df['time_offset'].max()
                min_time_offset = invalid_points_df['time_offset'].min()
                diff = max_time_offset - min_time_offset
                avg_time_step = diff / n_invalid_pts
                print(f"the maximum of the time_offset of the invalid points is: max={max_time_offset}")
                print(f"the minimum of the time_offset of the invalid points is: min={min_time_offset}")
                print(f"the diff of min-max the time_offset of the invalid points is: diff={diff}")
                print(f"the number of invalid points is: n_invalid_pts={n_invalid_pts}")
                print(f"the average time step between the invalid points is: avg_time_step={avg_time_step}")
                print(f"time_steps of the invalid points: \n{invalid_points_df['time_step'].value_counts()}")

                print(f"Conclusion: The 576 invalid points are evenly distributed across the 6 lasers: 96 each. \
                    \n They consist of equal numbers of first and second returns, because the time_step is either 0ns or 4167.0ns. \
                    \n However, the first and second returns cannot be differentiated by their tag, because the tag is always = 0 for these invalid points. \
                    \n ")
                
                # check out the problematic area of the invalid points. Where are they exatly located?
                min_idx = new_df.index[new_df['time_offset'] == min_time_offset].min()  # the min_idx would be a list of two indices bec. the time_offset is the same for the two returns
                max_idx = new_df.index[new_df['time_offset'] == max_time_offset].max()
                print(f"min index = {min_idx.astype(int)}")
                min_df = new_df.loc[min_idx-120:min_idx-1]  # df holding the 120 points before the min
                max_df = new_df.loc[max_idx+1:max_idx+120]
                invalids_by_idx_df = new_df.loc[min_idx:max_idx]  # df holding all invalid points
                edges_df = new_df.loc[[min_idx, max_idx]]  # df holding only the start and end point of the invalid points
                large_time_steps_df = new_df[new_df['time_step']>4167.0]

                # for comparsion the average time step between all points in the point cloud:
                max_time_offset = new_df['time_offset'].max()
                min_time_offset = new_df['time_offset'].min()
                diff = max_time_offset - min_time_offset
                n_all_pts = len(new_df)
                avg_time_step_ = diff / n_all_pts
                print("***************")
                print(f"the maximum of the time_offset of ALL points is: max={max_time_offset}")
                print(f"the minimum of the time_offset of ALL points is: min={min_time_offset}")
                print(f"the diff of min-max the time_offset of ALL points is: diff={diff}")
                print(f"the number of ALL points is: n_invalid_pts={n_all_pts}")
                print(f"the average time step between ALL points is: avg_time_step={avg_time_step_}")
                print(f"time_steps of ALL points: \n{new_df['time_step'].value_counts()}")

                # for comparison how the tag info looks like for valid no-hit points. 
                print(f"tag info of range_m = 0 points: \n{new_df[new_df['range_m']==0.0]['tag'].value_counts()}")
                # for comparison how the tag info looks like for valid hit points. 
                print(f"tag info of range_m > 0 points: \n{new_df[new_df['range_m']>0.0]['tag'].value_counts()}")
                # for comparison how the tag info looks like for all points. 
                print(f"tag info of ALL points: \n{new_df['tag'].value_counts()}")

                print(f"Frame {msg_idx} Count of 90° values: {new_df['el_deg'].value_counts().get(90.0,0)}")
                # print(new_df.info())
                # print(new_df.describe())
                # print(new_df)

                max_el = max(min_df['el_deg'].max().item(),max_df['el_deg'].max().item())
                min_el = min(min_df['el_deg'].min().item(),max_df['el_deg'].min().item())
                max_az = max(min_df['az_deg'].max().item(),max_df['az_deg'].max().item())
                min_az = min(min_df['az_deg'].min().item(),max_df['az_deg'].min().item())
                # print(f"plotting limits:\n elevation min:{min_el} max:{max_el} \n azimuth min:{min_az} max:{max_az}")

                ################# plot of pattern colored by time_offset #################
                ax = new_df.plot.scatter(x="az_deg", y="el_deg", c="time_offset", s=1, figsize=(7, 7))
                ax.set_aspect('equal')
                ax.set_title(f"Livox Tele-15: Gap in real Scanning Pattern of Frame {msg_idx}")
                ax.set_ylabel("Elevation [deg]")
                ax.set_xlabel("Azimuth [deg]")
                ax.set_ylim(min_el-1, max_el+1)
                ax.set_xlim(min_az-1, max_az+1)
                min_df.plot.scatter(x="az_deg", y="el_deg", c="b", s=1, ax=ax)
                max_df.plot.scatter(x="az_deg", y="el_deg", c="r", s=1, ax=ax)
                # invalids_by_idx_df.plot.scatter(x="az_deg", y="el_deg", c="g", s=2, figsize=(15,5), ax=ax)
                edges_df.plot.scatter(x="az_deg", y="el_deg", c="g", s=4, ax=ax)
                plt.tight_layout()
                filename = os.path.join(str(self._plotting_dir), f'Livox_Tele-15_pattern_Gap_in_frame_{msg_idx}.png')
                plt.savefig(filename, bbox_inches="tight")
                print(f"saving to {filename}")
                if self._show_plots:
                    plt.show()

                ################# plot of pattern colored by time_offset #################
                '''
                ax1 = new_df.plot.scatter(x="az_deg", y="el_deg", c="time_offset", s=1, figsize=(15,5))
                ax1.set_aspect('equal')
                ax1.set_title(f"Real Scanning Pattern of Livox Tele-15 Frame {msg_idx}")
                ax1.set_ylabel("Elevation [deg]")
                ax1.set_xlabel("Azimuth [deg]")
                ax1.set_xlim(-1,1)
                ax1.set_ylim(89,91)
                min_df.plot.scatter(x="az_deg", y="el_deg", c="b", s=1, ax=ax1)
                max_df.plot.scatter(x="az_deg", y="el_deg", c="r", s=1, ax=ax1)
                #invalids_by_idx_df.plot.scatter(x="az_deg", y="el_deg", c="g", s=2, ax=ax1)
                edges_df.plot.scatter(x="az_deg", y="el_deg", c="g", s=4, ax=ax1)
                '''

                ################# plot of line-no. over time_offset #################
                ax2 = new_df.plot.scatter(x="time_offset", y="line", c="time_offset", s=1, figsize=(7, 7))
                ax2.set_title(f"Livox Tele-15: Timestamps of invalid Null points in Frame {msg_idx}")
                ax2.set_ylabel("line")
                ax2.set_xlabel("time_offset [ns]")
                ax2.set_xlim(new_df.loc[min_idx-500]['time_offset'], new_df.loc[max_idx+500]['time_offset'])
                ax2.set_ylim(-1,6)
                min_df.plot.scatter(x="time_offset", y="line", c="b", s=1, ax=ax2)
                max_df.plot.scatter(x="time_offset", y="line", c="r", s=1, ax=ax2)
                invalids_by_idx_df.plot.scatter(x="time_offset", y="line", c="k", s=4, ax=ax2)
                edges_df.plot.scatter(x="time_offset", y="line", c="g", s=4, ax=ax2)
                filename = os.path.join(str(self._plotting_dir), f"Livox_Tele-15_timestamps_of_invalid_Null_points_in_frame_{msg_idx}.png")
                plt.savefig(filename)
                print(f"saving to {filename}")
                fig = px.scatter(new_df, x="time_offset", y="line", title=f"Livox Tele-15: Timestamps of invalid Null points in Frame {msg_idx}",)
                fig.add_trace(go.Scatter(x=large_time_steps_df["time_offset"], y=large_time_steps_df["line"], mode='markers',
                                         name="time_step > 4167ns",
                                         marker=dict(color='MediumPurple', size=10)))
                filename = os.path.join(str(self._plotting_dir), f"Livox_Tele-15_timestamps_of_invalid_Null_points_in_frame_{msg_idx}.html")
                fig.write_html(filename)
                print(f"saving to {filename}")
                if self._show_plots:
                    fig.show()

                ################# plot of pattern colored by time_step #################
                ax3 = new_df.plot.scatter(x="az_deg", y="el_deg", c="time_offset", s=1, figsize=(7, 7))
                ax3.set_aspect('equal')
                ax3.set_title(f"Livox Tele-15: Timestamps before and after invalid Null points in Frame {msg_idx}")
                ax3.set_ylabel("Elevation [deg]")
                ax3.set_xlabel("Azimuth [deg]")
                ax3.set_ylim(min_el-1, max_el+1)
                ax3.set_xlim(min_az-1, max_az+1)
                min_df.plot.scatter(x="az_deg", y="el_deg", c="b", s=1, ax=ax3, label="Timestamp before invalid Null points")
                max_df.plot.scatter(x="az_deg", y="el_deg", c="r", s=1, ax=ax3, label="Timestamp after invalid Null points")
                # invalids_by_idx_df.plot.scatter(x="az_deg", y="el_deg", c="g", s=2, ax=ax3)
                # edges_df.plot.scatter(x="az_deg", y="el_deg", c="g", s=4, ax=ax3)
                large_time_steps_df.plot.scatter(x="az_deg", y="el_deg", c="g", s=4, ax=ax3, label="Points with large timestamp jumps")
                plt.tight_layout()
                filename = os.path.join(str(self._plotting_dir), f'Livox_Tele-15_pattern_timestamps_before_and_after_invalid_Null_points_in_frame_{msg_idx}.png', bbox_inches="tight")
                plt.savefig(filename)
                print(f"saving to {filename}")

        else:  # this is just for visualization of the old pointcloud content without spherical coordinate information.
            new_df = self._msgToDataFrame(msg)
            # print(new_df.info())
            # print(new_df.describe())
            # print(new_df)

            ax = new_df.plot.scatter(x="az_deg", y="el_deg", c="range_m", s=1, figsize=(7,7))
            ax.set_aspect('equal')
            ax.set_title(f"Real Scanning Pattern of Livox Tele-15 Frame {msg_idx}")
            ax.set_ylabel("Elevation [deg]")
            ax.set_xlabel("Azimuth [deg]")
            # ax.set_xlim(-10,10)
            # ax.set_ylim(-10,10)
            if self._benchmark_driver:
                filename = os.path.join(str(self._plotting_dir), f'Livox_Tele-15_pattern_with_benchmark_driver_frame_{msg_idx}.png')
                plt.savefig(filename)
                print(f"saving to {filename}")
            elif self._proprietary_driver:
                filename = os.path.join(str(self._plotting_dir), f'Livox_Tele-15_pattern_with_proprietary_driver_frame_{msg_idx}.png')
                plt.savefig(filename)
                print(f"saving to {filename}")
            else:
                filename = os.path.join(str(self._plotting_dir), f'Livox_Tele-15_pattern_original_driver_without_spherical_point_information_frame_{msg_idx}.png')
                plt.savefig(filename)
                print(f"saving to {filename}")
            if self._show_plots:
                plt.show()


class udpPacketLossAnalysis:
    def __init__(self):
        pass
    
    # ["/livox/lidar_1PQDLCQ00139691"]                   # local measurements at Markus' workstation
    # ["/lvx/1PQDH5B00102851/points_raw"]                # GAF data
    # ["/long_range_tof_lidar_center/lidar/pointcloud"]  # project data

    def processRecording(self, path: str = "",
                         spherical_points_available: bool = True,
                         detect_missing_points: bool = True,
                         detect_time_jumps_between_frames: bool = False,
                         benchmark_driver: bool = False,
                         proprietary_driver: bool = False,
                         do_plotting: bool = False,
                         topics_to_analyse=["/livox/lidar_1PQDLCQ00139691"]):
        
        realROS2Recording = recording(path,
                                      spherical_points_available,
                                      detect_missing_points,
                                      detect_time_jumps_between_frames,
                                      benchmark_driver,
                                      proprietary_driver,
                                      do_plotting,
                                      topics_to_analyse)

