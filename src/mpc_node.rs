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


extern crate ackermann_msgs;
extern crate ament_rs;
extern crate geometry_msgs;
extern crate mpc_controller;
#[macro_use]
extern crate ndarray;
extern crate optimization_engine;


use std::convert::{TryFrom};
use std::env;
use std::sync::{Arc, Mutex};
use std::time::SystemTime;

use ackermann_msgs::msg::AckermannDriveStamped;
use geometry_msgs::msg::Pose;
use mpc_controller::solve;
use ndarray::prelude::*;
use optimization_engine::alm::{AlmCache};

/// Return euler angle from quaternion
fn quaternion_to_yaw(quat: &geometry_msgs::msg::Quaternion) -> f64 {
    let q0 = quat.x;
    let q1 = quat.y;
    let q2 = quat.z;
    let q3 = quat.w;

    (2.0 * (q2 * q3 + q0 * q1)).atan2(1.0 - 2.0 * (q1 * q1 + q2 * q2))
}

/// Transform Array2<f64> of poses into the poses field of PoseArray msg
fn arr_to_pose_array_msg(arr: &Array2<f64> , msg: &mut geometry_msgs::msg::PoseArray){

    for row in arr.outer_iter() {
        let mut pose = Pose::default();
        pose.position.x = row[0];
        pose.position.y = row[1];

        pose.orientation.w = (row[2] / 2.).cos();
        pose.orientation.z = (row[2] / 2.).sin();
        pose.orientation.y = 0.0;
        pose.orientation.x = 0.0;
        msg.poses.push(pose);
    }
    msg.header.frame_id = String::from("map");
    let time = SystemTime::now();

    msg.header.stamp.sec = i32::try_from(time.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_secs()).unwrap();
}

#[derive(Clone)]
struct Mpc {
    odom_data: Arc<Mutex<nav_msgs::msg::Odometry>>,
    raceline_data: Arc<Mutex<ndarray::Array2<f64>>>,

    // Mpc parameters
    horizon_length: usize,
    next_states: ndarray::Array2<f64>,
    u: ndarray::Array2<f64>,

    raceline_fn: String,
}

impl Mpc {
    pub unsafe fn new() -> Self {
        let horizon_length = 10;
        let ament = ament_rs::Ament::new().unwrap();

        let config_fn = ament.get_package_share_directory("rust_mpc").unwrap().to_str().unwrap().to_owned()
            + &String::from("/config/sim.yaml");

        let f = std::fs::File::open(config_fn).unwrap();
        let data: serde_yaml::Value = serde_yaml::from_reader(f).unwrap();
        let map_name = data["rust_mpc"]["ros__parameters"]["map"]
            .as_str()
            .map(|s| s.to_string())
            .unwrap() + "_centerline.csv";

        let raceline_fn = ament.get_package_share_directory("rust_mpc").unwrap().to_str().unwrap().to_owned()
            + &String::from("/data/") + &map_name;


        Self {
            odom_data: Arc::new(Mutex::new(nav_msgs::msg::Odometry::default())),
            raceline_data: Arc::new(Mutex::new(ndarray::Array2::default(ndarray::Dim([1, 1])))),
            horizon_length: horizon_length,
            next_states: ndarray::Array::ones((horizon_length, 3)),
            u: ndarray::Array::zeros((horizon_length, 2)),
            raceline_fn: raceline_fn,
        }
    }

    /// Load global raceline trajectory
    fn load_raceline(&self) {
        let mut rdr = csv::ReaderBuilder::new()
            .delimiter(b',')
            .from_reader(std::fs::File::open(self.raceline_fn.clone()).unwrap());
        let mut prev_x: f64 = rdr.records().next().unwrap().unwrap()[0].replace(" ", "").parse().unwrap();
        let mut prev_y: f64 = rdr.records().next().unwrap().unwrap()[1].replace(" ", "").parse().unwrap();
        let vec = rdr.records()
            .map(|rec| {
                let rec = rec.unwrap();
                let x: f64 = rec[0].replace(" ", "").parse().unwrap();
                let y: f64 = rec[1].replace(" ", "").parse().unwrap();
                let orientation = (y - prev_y).atan2(x - prev_x);
                prev_y = y;
                prev_x = x;
                array![x, y, orientation]
            })
            .collect::<Vec<Array1<f64>>>();

        let inner_shape = vec[0].dim();
        let shape = (vec.len(), inner_shape);
        let flat: Vec<f64> = vec.iter().flatten().cloned().collect();
        *self.raceline_data.lock().unwrap() = Array2::from_shape_vec(shape, flat).unwrap();
    }

    /// Takes raceline and position of ego-vehicle and computer n_of_points closest waypoints in forward direction
    pub fn desired_traj(&self, n_of_points: usize) -> (Array2<f64>, Array2<f64>) {

        // Compute closest point on path to vehicle
        // First get the matrix of differences between path points and our current position
        let raceline_path = &*self.raceline_data.lock().unwrap();
        let odom_data = &*self.odom_data.lock().unwrap();
        let s: i32 = 2;
        let x_and_y = &raceline_path.slice(s![..,..s]).to_owned();
        let car_pos_mx = array![odom_data.pose.pose.position.x, odom_data.pose.pose.position.y];

        let euclidean_dist_mx = (x_and_y - &car_pos_mx).mapv(|e| e.powi(2)).sum_axis(Axis(1));

        let min_idx = euclidean_dist_mx.iter().enumerate().fold((0, euclidean_dist_mx[0]), |min, (ind, &val)| if val < min.1 { (ind, val) } else { min }).0;

        let slice_end = min_idx + n_of_points;
        let mut x;
        if slice_end > raceline_path.dim().0 {
            x = raceline_path.slice(s![0..(n_of_points), ..]).to_owned();
        } else {
            x = raceline_path.slice(s![min_idx..(slice_end), ..]).to_owned();
        }
        x.row_mut(0).assign(&array![odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, quaternion_to_yaw(&odom_data.pose.pose.orientation)]);
        let mut prev_x = *x.row(0).get(0).unwrap();
        let mut prev_y = *x.row(0).get(1).unwrap();
        let mut prev_theta = *x.row(0).get(2).unwrap();
        let mut dx = x.clone();
        let mut cntr_row = 0;
        for row in x.outer_iter() {
            let x: f64 = *row.get(0).unwrap();
            let y: f64 = *row.get(1).unwrap();
            let theta = *row.get(2).unwrap();

            dx.row_mut(cntr_row).assign(&array![x-prev_x, y-prev_y, theta-prev_theta]);
            prev_x = x;
            prev_y = y;
            prev_theta = theta;
            cntr_row += 1;
        };
        let omega_col: i32 = 2;
        let y_col: i32 = 1;
        let x_col: i32 = 0;

        let dx_x = &dx.slice(s![..,x_col]).to_owned();
        let dx_y = &dx.slice(s![..,y_col]).to_owned();


        let omega = &dx.slice(s![..,omega_col]).to_owned();
        let vx = dx_x * omega.mapv(|e| e.cos()) + dx_y * omega.mapv(|e| e.sin());
        let vy = dx_x * omega.mapv(|e| e.sin()) - dx_y * omega.mapv(|e| e.cos());
        let v = vx.mapv(|e| e.powi(2)) + vy.mapv(|e| e.powi(2));

        let u = stack![Axis(1), v.view(), omega.view()];

        (x.clone(), u)
    }

    pub fn callback(&self, msg: nav_msgs::msg::Odometry) {
        *self.odom_data.lock().unwrap() = msg;
    }

    pub unsafe fn compute_control(&mut self, cache: &mut AlmCache) -> (ndarray::Array2<f64>, ndarray::Array2<f64>) {
        let (states_ref, controls_ref) = self.desired_traj(self.horizon_length);

        let states_ref_flat = Array::from_iter(states_ref.into_iter());
        let mut controls_ref_flat = Array::from_iter(controls_ref.into_iter());


        let st_iter = self.next_states.clone().into_iter();
        let c_iter = self.u.clone().into_iter();
        let states_flat = Array::from_iter(st_iter);
        let mut controls_flat = Array::from_iter(c_iter);


        controls_ref_flat.append(Axis(0), states_ref_flat.view()).unwrap();
        controls_flat.append(Axis(0), states_flat.view()).unwrap();
        let u = &mut controls_flat.to_vec()[..];
        let x: &[f64] = &mut controls_ref_flat.to_vec()[..];



        match solve(x, cache, u.as_mut(), &None, &std::option::Option::from(0.1)) {
            Err(e) => println!("{:?}", e),
            _ => ()
        }

        let u_computed = &u[..20];
        let x_computed = &u[20..50];

        let n_states = ndarray::Array::from_shape_vec(self.next_states.dim(), x_computed.to_vec()).unwrap();
        let u_ref = ndarray::Array::from_shape_vec(self.u.dim(), u_computed.to_vec()).unwrap();

        if !u_ref[[0, 0]].is_nan() {
            self.next_states = n_states.clone();
            self.u = u_ref.clone();
        }
        (self.next_states.clone(), self.u.clone())
    }
}

struct MpcNode {
    mpc: Mpc,
    // currently unused
    node: rclrs::node::Node,
    // currently unused
    _odom_subscription: Arc<rclrs::node::Subscription<nav_msgs::msg::Odometry>>,
    raceline_publisher: rclrs::node::Publisher<geometry_msgs::msg::PoseArray>,
}

impl MpcNode {


    pub unsafe fn new(context: &rclrs::context::Context) -> Result<Self, anyhow::Error> {
        let mut node = context.create_node("mpc_node")?;

        let mpc = Mpc::new();
        mpc.load_raceline();

        let raceline_publisher = node.create_publisher::<geometry_msgs::msg::PoseArray>(
            "raceline", rclrs::QOS_PROFILE_DEFAULT,
        )?;
        let desired_raceline_publisher = node.create_publisher::<geometry_msgs::msg::PoseArray>(
            "desired_raceline", rclrs::QOS_PROFILE_DEFAULT,
        )?;

        let drive_pub = node.create_publisher::<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", rclrs::QOS_PROFILE_DEFAULT,
        )?;
        let _odom_subscription = {
            let mut mpc = mpc.clone();
            node.create_subscription::<nav_msgs::msg::Odometry, _>(
                "ego_racecar/odom",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: nav_msgs::msg::Odometry| {
                    mpc.callback(msg);

                    let mut cache = mpc_controller::initialize_solver();

                    std::thread::sleep(std::time::Duration::from_millis(60));
                    let (states, controls) = mpc.compute_control(&mut cache);

                    let mut msg = geometry_msgs::msg::PoseArray::default();
                    let mut ack_msg = AckermannDriveStamped::default();
                    arr_to_pose_array_msg(&states, &mut msg);

                    ack_msg.header = msg.header.clone();

                    // Calculate steering from angular velocity based on ackermann model
                    let vel = controls.row(0).get(0).unwrap().clone() as f32;
                    let angular_vel = controls.row(0).get(1).unwrap().clone() as f32;
                    ack_msg.drive.speed = vel;
                    let wheelbase = 0.3302;
                    let angle = ((angular_vel / vel) * wheelbase).atan();
                    ack_msg.drive.steering_angle = angle;

                    match drive_pub.publish(ack_msg) {
                        Err(e) => println!("{:?}", e),
                        _ => ()
                    }
                    match desired_raceline_publisher.publish(msg) {
                        Err(e) => println!("{:?}", e),
                        _ => ()
                    }


                },
            )?
        };


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

        arr_to_pose_array_msg(& *self.mpc.raceline_data.lock().unwrap(), &mut msg);

        match self.raceline_publisher.publish(msg) {
            Err(e) => println!("{:?}", e),
            _ => ()
        }
    }

    pub fn spin(&self) -> Result<(), anyhow::Error> {
        rclrs::spin(&self.node).map_err(|err| err.into())
    }
}

fn main() -> Result<(), anyhow::Error> {
    let context = rclrs::Context::new(env::args())?;
    let mpc_node;
    unsafe {
        mpc_node = MpcNode::new(&context)?;
    }
    mpc_node.publish_raceline();
    mpc_node.spin()
}