#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include <laser_vel_filter/mode.h>

class Laser_vel_filter
{
    private:
        ros::NodeHandle n_;
        ros::Publisher pub_max_vel_;
        ros::Subscriber sub_scan_;
        ros::ServiceServer srv_mode_;

        sensor_msgs::LaserScan from_scan_;

        bool scan_comeup_ = false;
        int mode_;
        int beam_detect = 0;
        int decided_threshold = 30;

        struct data_zone
        {
            int start_scan;
            int stop_scan;
            float threshold_dist_;
            float max_vel_;
        };

        data_zone red_zone, yellow_zone, green_zone;

        enum ZONE{  INIT,
                    RED_ZONE,
                    YELLOW_ZONE,
                    GREEN_ZONE
                    };
        
        enum MODE{  DISABLE,
                    ENABLE
                 };


    public:
        Laser_vel_filter()
        {
            pub_max_vel_ = n_.advertise<std_msgs::Float32>("/max_vel", 1);

            sub_scan_ = n_.subscribe("/scan", 1, &Laser_vel_filter::cbScan, this);

            srv_mode_ = n_.advertiseService("/laser_vel_filter/mode", &Laser_vel_filter::cbMode, this);

            red_zone.start_scan = 113; 
            red_zone.stop_scan = 590;
            red_zone.threshold_dist_ = 0.8;
            red_zone.max_vel_ = 0.30;

            yellow_zone.start_scan = 261;
            yellow_zone.stop_scan = 445;
            yellow_zone.threshold_dist_ = 1.7;
            yellow_zone.max_vel_ = 0.5;

            green_zone.start_scan = 0;
            green_zone.stop_scan = 0;
            green_zone.threshold_dist_ = 0;
            green_zone.max_vel_ = 1.0;

            //ROS_INFO("Start Laser_vel_filter");
            
            ros::Rate loop_rate(50);

            while(ros::ok())
            {
                if((mode_ == MODE::ENABLE) && (scan_comeup_))
                {
                    ROS_INFO_ONCE("Start Laser_vel_filter");
                    update_max_vel();
                }
                else
                {
                    ROS_ERROR_ONCE("wait scan comeup or Enable mode");
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        bool cbMode(laser_vel_filter::mode::Request &req, laser_vel_filter::mode::Response &res)
        {
            mode_ = (req.mode == MODE::ENABLE) ? MODE::ENABLE : MODE::DISABLE;
            res.result = 1;

            return true;
        }

        void cbScan(const sensor_msgs::LaserScan& scan_msg)
        {
            from_scan_ = scan_msg;
            scan_comeup_ = true;
        }

        int obstacle_zone_detected()
        {
            int zone_search = ZONE::RED_ZONE;
            int zone_deteced = ZONE::INIT;
            int return_zone = 0;
            beam_detect = 0;

            for(int i = 0; i<=1 ; i++)
            {
                switch(zone_search)
                {
                    case ZONE::RED_ZONE:
                        // search obstacle in **redzone 
                        for(int i = red_zone.start_scan; i <= red_zone.stop_scan ; i++)
                        {
                            //printf("zone_red searching\n");
                            // if(from_scan_.ranges[i] <= red_zone.threshold_dist_)
                            if(from_scan_.ranges[i] <= 0.5)
                            {
                                beam_detect++;
                                //printf("zone_red detect\n");
                            }
                        }
                        zone_deteced = ZONE::RED_ZONE;
                        zone_search = ZONE::YELLOW_ZONE;
                        break;
                    
                    case ZONE::YELLOW_ZONE:
                        // search obstacle in **yellowzone 
                        for(int i = yellow_zone.start_scan; i <= yellow_zone.stop_scan ; i++)
                        {
                            //printf("zone_yellow searching\n");
                            if((from_scan_.ranges[i] <= yellow_zone.threshold_dist_) && (from_scan_.ranges[i] >= red_zone.threshold_dist_))
                            {
                                beam_detect++;
                            }
                        }
                        zone_deteced = ZONE::YELLOW_ZONE;
                        break;

                }


                if(beam_detect >= decided_threshold)
                {
                    return_zone = zone_deteced;
                    // beam_detect = 0;
                    // zone_deteced = ZONE::INIT;
                    //printf("beam_detect = %d : zone_detect = %d\n", beam_detect, zone_deteced);
                    break;
                }
                else
                {
                    return_zone = ZONE::GREEN_ZONE;
                    //printf("zone_green detect\n");
                }

                beam_detect = 0;
                zone_deteced = ZONE::INIT;
                //printf("beam_detect = %d : zone_detect = %d\n", beam_detect, zone_deteced);

                
            }

            //printf("return_zone : %d\n" , return_zone);
            return return_zone;
            
        }

        void update_max_vel()
        {
            

            int car_state = obstacle_zone_detected();
            std_msgs::Float32 max_vel;

            if(car_state == ZONE::RED_ZONE)
            {
                max_vel.data = red_zone.max_vel_;
            }
            else if(car_state == ZONE::YELLOW_ZONE)
            {
                max_vel.data = yellow_zone.max_vel_;
            }
            else if(car_state == ZONE::GREEN_ZONE)
            {
                max_vel.data = green_zone.max_vel_;
            }
            
            // ROS_INFO("Found obstacle in zone : %d", car_state);
            pub_max_vel_.publish(max_vel);

        }

};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "Laser_vel_filter");

    
    
    //Create an object of class SubscribeAndPublish that will take care of everything
    Laser_vel_filter runObject;

    ros::spin();

    return 0;
}