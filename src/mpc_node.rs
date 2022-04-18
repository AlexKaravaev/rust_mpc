/*
 * Copyright: 2022 Alex Karavaev <alexkaravev@gmail.com>
 * License: MIT
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense,
 *  and/or sell copies of the Software, and to permit persons to whom the
 *  Software is furnished to do so, subject to the following conditions:
 *  .
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *  .
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


use std::convert::{TryFrom, TryInto};
use std::env;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;
use ament_rs::ament;
use geometry_msgs::msg::Pose;
use ndarray::Dim;

// use nav_msgs::msg::Odometry;
// use anyhow::{Error, Result};
#[macro_use]
extern crate ndarray;
extern crate geometry_msgs;
extern crate ament_rs;

use ndarray::prelude::*;

// Return euler angle from quaternion
fn quaternion_to_yaw(quat: geometry_msgs::msg::Quaternion) -> f64 {
    let q0 = quat.x;
    let q1 = quat.y;
    let q2 = quat.z;
    let q3 = quat.w;

    (2.0 * (q2 * q3 * q0 * q1)).atan2(1.0 - 2.0 * (q1 * q1 + q2 * q2))
}

#[derive(Clone)]
struct Mpc {
    odom_data: Arc<Mutex<nav_msgs::msg::Odometry>>,
    raceline_data: Arc<Mutex<ndarray::Array2<f64>>>,

    // Mpc parameters
    timestep: f64,
    horizon_length: usize,

    //  Weights matricies
    Q: ndarray::Array1<f64>,
    R: ndarray::Array1<f64>,

    // Constraints for v and omega and it's derivatives
    min_v: f64,
    max_v: f64,

    min_omega: f64,
    max_omega: f64,

    max_dv: f64,
    max_domega: f64,

    next_states: ndarray::Array2<f64>,
    u: ndarray::Array2<f64>,
    raceline_fn: String,

}

impl Mpc {
    pub fn new() -> Self {
        let horizon_length = 10;
        let ament = ament_rs::Ament::new().unwrap();
        let raceline_fn = ament.get_package_share_directory("rust_mpc").unwrap().to_str().unwrap().to_owned()
            + &String::from("/data/Silverstone_centerline.csv");

        Self {
            odom_data: Arc::new(Mutex::new(nav_msgs::msg::Odometry::default())),
            raceline_data: Arc::new(Mutex::new(ndarray::Array2::default(ndarray::Dim([1, 1])))),
            timestep: 0.06,
            horizon_length: horizon_length,
            Q: ndarray::Array1::from(vec![50., 50., 2.]),
            R: ndarray::Array1::from(vec![1., 1.]),
            min_v: -5.0,
            max_v: 5.0,
            min_omega: -3.0,
            max_omega: 3.0,
            max_dv: 1.0,
            max_domega: std::f64::consts::PI / 3.0,
            next_states: ndarray::Array::ones((horizon_length + 1, 3)),
            u: ndarray::Array2::zeros([horizon_length, 2]),
            raceline_fn: raceline_fn,
        }
    }

    /// Load global raceline trajectory
    fn load_raceline(&self){
        let mut rdr = csv::ReaderBuilder::new()
            .delimiter(b',')
            .from_reader(std::fs::File::open(self.raceline_fn.clone()).unwrap());
        println!("{:?}", rdr.records().next().unwrap());
        let mut prev_x:f64 = rdr.records().next().unwrap().unwrap()[0].replace(" ", "").parse().unwrap();
        let mut prev_y:f64 = rdr.records().next().unwrap().unwrap()[1].replace(" ", "").parse().unwrap();
        let vec = rdr.records()
            .map(|mut rec| {
                let rec = rec.unwrap();
                let x:f64 = rec[0].replace(" ", "").parse().unwrap();
                let y:f64 = rec[1].replace(" ", "").parse().unwrap();
                let orientation = (y - prev_y).atan2(x - prev_x);
                // let orientation = rec[3].parse().unwrap();
                prev_y = y;
                prev_x = x;
                array![x, y, orientation]
            })
            .collect::<Vec<Array1<f64>>>();

        let inner_shape = vec[0].dim();
        let shape = (vec.len(), inner_shape);
        let flat: Vec<f64> = vec.iter().flatten().cloned().collect();
        // println!("{} {}", shape.0, shape.1);
        *self.raceline_data.lock().unwrap() = Array2::from_shape_vec(shape, flat).unwrap();

    }


    /// Takes raceline and position of ego-vehicle and computer n_of_points closest waypoints in forward direction
    pub fn desired_traj(&self, n_of_points: i64, path: geometry_msgs::msg::PoseArray) {

    }

    pub fn callback(&self, msg: nav_msgs::msg::Odometry) {
        *self.odom_data.lock().unwrap() = msg;
    }

    pub fn compute_control(&self) {
        for n in 0..self.raceline_data.lock().unwrap().len(){
            println!("Raceline: {:?}", *self.raceline_data.lock().unwrap());
        }
        // println!("Latest value: {:?}", *self.odom_data.lock().unwrap());
    }
}

struct MpcNode {
    mpc: Mpc,
    // currently unused
    node: rclrs::node::Node,
    // currently unused
    _odom_subscription: Arc<rclrs::node::Subscription<nav_msgs::msg::Odometry>>,
    raceline_publisher: rclrs::node::Publisher<geometry_msgs::msg::PoseArray>
}

impl MpcNode {
    pub fn new(context: &rclrs::context::Context) -> Result<Self, anyhow::Error> {
        let mut node = context.create_node("mpc_node")?;

        let mpc = Mpc::new();
        mpc.load_raceline();
        let _odom_subscription = {
            let mpc = mpc.clone();
            node.create_subscription::<nav_msgs::msg::Odometry, _>(
                "ego_racecar/odom",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: nav_msgs::msg::Odometry| {
                    mpc.callback(msg);
                },
            )?
        };

        // let _raceline_subscription = {
        //     let mpc = mpc.clone();
        //     node.create_subscription::<geometry_msgs::msg::PoseArray, _>(
        //         "raceline",
        //         rclrs::QOS_PROFILE_DEFAULT,
        //         move |msg: geometry_msgs::msg::PoseArray| {
        //             // Transform PoseArray to Array with [x,y,orientation] elems
        //             let pose_vec = msg.poses
        //                 .iter()
        //                 .map(|pose| array![pose.position.x, pose.position.y, quaternion_to_yaw(pose.orientation.clone())])
        //                 .collect::<Vec<Array1<f64>>>();
        //             let inner_shape = pose_vec[0].dim();
        //             let shape = (pose_vec.len(), inner_shape);
        //             let flat: Vec<f64> = pose_vec.iter().flatten().cloned().collect();
        //
        //             *mpc.raceline_data.lock().unwrap() = Array2::from_shape_vec(shape, flat).unwrap();
        //
        //         },
        //     )?
        // };
        let raceline_publisher = node.create_publisher::<geometry_msgs::msg::PoseArray>(
                "raceline", rclrs::QOS_PROFILE_DEFAULT
            )?;




        // Currently, ROS timers are still unimplemented, so we need a separate "timer" thread
        {
            let mpc = mpc.clone();
            std::thread::spawn(move || {
                loop {
                    std::thread::sleep(std::time::Duration::from_millis(100));
                    mpc.compute_control();
                }
            });
        }

        Ok(Self {
            mpc,
            node,
            _odom_subscription,
            raceline_publisher,
        })
    }

    /// Publish raceline as PoseArray for visualization
    fn publish_raceline(&self) {
        let mut msg = geometry_msgs::msg::PoseArray::default();

        for row in self.mpc.raceline_data.lock().unwrap().outer_iter(){
            let mut pose = Pose::default();
            pose.position.x = row[0];
            pose.position.y = row[1];
            // qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            // qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            // qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            // qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            pose.orientation.w = (row[2]/2.).cos();
            pose.orientation.z = (row[2]/2.).sin();
            pose.orientation.y = 0.0;
            pose.orientation.x = 0.0;
            msg.poses.push(pose);

        }
        msg.header.frame_id = String::from("map");
        let time = SystemTime::now();
        println!("{}",time.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_secs());
        msg.header.stamp.sec = i32::try_from(time.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_secs()).unwrap();
        // msg.header.stamp.nanosec = u32::try_from(time.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_nanos()).unwrap();

        self.raceline_publisher.publish(msg);

        println!("Published!");
    }

    pub fn spin(&self) -> Result<(), anyhow::Error> {
        rclrs::spin(&self.node).map_err(|err| err.into())
    }
}

fn main() -> Result<(), anyhow::Error> {
    let context = rclrs::Context::new(env::args())?;
    let mpc_node = MpcNode::new(&context)?;

    mpc_node.publish_raceline();
    mpc_node.spin()
}