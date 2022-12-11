use std::path::Path;

use iterative_closest_point_registration::icp::IterativeClosestPoint;
use navigation_core::ports::{odometry::Odometry, sensor_input::SensorInput};
use pointcloud_drivers::robotcar_dataset::RobotCarExperience;

fn main() {
    let root_dir = Path::new(
        "/home/daneel/data/rrcd_part/2019-01-10-14-36-48-radar-oxford-10k-partial/velodyne_left",
    );
    let mut roboexp = RobotCarExperience::new(&root_dir);
    roboexp.read_data().unwrap();

    let mut i = 0;
    let mut odometry = IterativeClosestPoint::new(1e-5, 100);
    while let Some(data) = roboexp.read_data() {
        let trans = odometry.get_odometry(&data);
        println!("{:?}", trans);

        i += 1;
        if i == 100 {
            break;
        }
    }
}
