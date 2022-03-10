#include "ros/ros.h"
#include "firefly_mapping/ImageWithPose.h"
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/UInt8MultiArray.h>


class FireflyMapping {

public:
    FireflyMapping() {
        image_sub = nh.subscribe("image_to_project", 1000, &FireflyMapping::project_image, this);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("observed_firemap", 10);

        K_inv << 1.0/fx,  0.0,    -cx/fx,
                0.0,     1.0/fy, -cy/fy,
                0.0,     0.0,     1.0;

        map.header.frame_id = "world";
        map.info.resolution = 0.5;
        map.info.width = 400; //Number of Cells
        map.info.height = 400; //Number of Cells
        map.info.origin.position.x = -100; //In meters
        map.info.origin.position.y = -100; //In meters
        map.data = std::vector<std::int8_t> (400*400, -1);

        std::cout << "Hello There" << std::endl;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Publisher map_pub;

    nav_msgs::OccupancyGrid map;

    Eigen::Vector3d ground_normal{0, 0, 1}; //Should point up from ground - if pointing into ground, will cause errors
    float ground_offset = 0;

    float fx = 411.3366;
    float fy = 412.6712;
    float cx = 320.4049;
    float cy = 251.967;

    float resolution = 0.5;
    float minX = -100;
    float maxX = 100;
    float minY = -100;
    float maxY = 100;

    Eigen::Matrix3d K_inv;

    void project_image(const firefly_mapping::ImageWithPose& msg) {
        Eigen::Quaterniond cam_quat;
        tf::quaternionMsgToEigen(msg.pose.orientation, cam_quat);

        for (size_t i = 0; i < msg.image.width; i++) {
            for (size_t j = 0; j < msg.image.height; j++) {
                uint8_t pixelValue = msg.image.data[i + j * msg.image.width];
//                if (pixelValue == 0) {
//                    continue;
//                }
                Eigen::Vector3d pixelHomogenous{i, j, 1};

                Eigen::Vector3d rayDir = cam_quat.matrix() * K_inv * pixelHomogenous;
                Eigen::Vector3d camCenter {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};

                float rayPerpendicularComponent = ground_normal.dot(rayDir);

                if (rayPerpendicularComponent >= 0) {
                    continue;
                }

                float camCenterGroundDist = ground_offset - ground_normal.dot(camCenter);

                float rayLambda = camCenterGroundDist / rayPerpendicularComponent;

                Eigen::Vector3d intersect = camCenter + rayLambda * rayDir;

                if (intersect(0) > maxX
                    || intersect(0) < minX
                    || intersect(1) > maxY
                    || intersect(1) < minY) {
                    continue;
                }

                size_t gridRow = (size_t) ((intersect(1)-minY)/resolution);
                size_t gridCol = (size_t) ((intersect(0)-minX)/resolution);

                if (pixelValue == 0) {
                    map.data[gridCol + gridRow * map.info.width] = 0;
                }
                else {
                    map.data[gridCol + gridRow * map.info.width] = 100;
                }

            }
        }

        for (int i = 0; i < 100; i++) {
            map.data[i] = 100;
        }
        map_pub.publish(map);
        return;
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "firefly_mapping");
    FireflyMapping node;
    ros::spin();
    return 0;
}