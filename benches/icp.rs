use std::path::Path;

use criterion::{criterion_group, criterion_main, Criterion};
use iterative_closest_point_registration::{icp::IterativeClosestPoint, pointcloud::PointCloudXYZ};
use navigation_core::ports::{odometry::Odometry, sensor_input::SensorInput};
use pointcloud_drivers::robotcar_dataset::RobotCarExperience;

fn robotcar_read_data(c: &mut Criterion) {
    let root_dir = Path::new(
        "/home/daneel/data/rrcd_part/2019-01-10-14-36-48-radar-oxford-10k-partial/velodyne_left",
    );
    let mut roboexp = RobotCarExperience::new(&root_dir);
    roboexp.read_data().unwrap();
    let data = roboexp.read_data().unwrap();

    let mut group = c.benchmark_group("sample_data_robotcar");
    group.bench_function("read_one_data", |b| b.iter(|| PointCloudXYZ::from(&data)));
    group.finish();
}

fn robotcar_icp(c: &mut Criterion) {
    let root_dir = Path::new(
        "/home/daneel/data/rrcd_part/2019-01-10-14-36-48-radar-oxford-10k-partial/velodyne_left",
    );
    let mut roboexp = RobotCarExperience::new(&root_dir);
    roboexp.read_data().unwrap();
    let data1 = roboexp.read_data().unwrap();
    let data2 = roboexp.read_data().unwrap();

    let mut odometry = IterativeClosestPoint::new(1e-5, 100);

    let mut group = c.benchmark_group("icp_robotcar");
    group.bench_function("get_icp", |b| {
        b.iter(|| {
            odometry.get_odometry(&data1);
            odometry.get_odometry(&data2);
        })
    });
    group.finish();
}

criterion_group!(benches, robotcar_read_data, robotcar_icp);
criterion_main!(benches);
