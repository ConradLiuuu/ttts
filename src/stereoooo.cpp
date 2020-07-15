#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "Eigen/Eigen"
#include <iostream>
#include <math.h>
#include <vector>
#include <thread>
#include <fstream>

using namespace std;
using namespace ros;
using namespace Eigen;

typedef Matrix <double, 1, 8> RowVector8i;
//typedef Matrix <double, 1, 3> RowVector3ii;

class Stereo
{
private:
    //Eigen::Matrix <double, 1, 3> A;
    //RowVector3ii A;
    
    string camera_L, camera_R;

    // left camera intrinsic parameters
    double fu_L, fv_L, u0_L, v0_L;
    //double kc_L[8];
    RowVector8i kc_L;

    // right camera intrinsic parameters
    double fu_R, fv_R, u0_R, v0_R;
    //double kc_R[8];
    RowVector8i kc_R;

    int dis_Ix_L, dis_Iy_L, dis_Ix_R, dis_Iy_R;
    double Ix_L, Iy_L, Ix_R, Iy_R;

    int ID_L, ID_R, id_L, id_R;
    int ID_L_past, ID_R_past;

    int contour_L, contour_R;

    // right to left extrinsic params
    Matrix <double, 3, 3> R_R2L;
    Matrix <double, 3, 1> b_R2L;
    Matrix <double, 3, 1> b_L2R;
    // left to world extrinsic parmas
    Matrix <double, 3, 1> d;
    Matrix <double, 3, 1> b_L2W;
    Matrix <double, 3, 3> R_W2L;
    Matrix <double, 3, 1> h_L;
    Matrix <double, 3, 1> h_W;
    Matrix <double, 3, 1> compesation_value;

    // ray vectors
    double hx_L, hy_L, hz_L;
    double hx_W, hy_W, hz_W;
    double hx, hy, hz;

    double start_,end_;

    double k;

    //int y1, y2, z1, z2, i;
    int i;


    // ros setting
    NodeHandle nh;
    Subscriber sub_left, sub_right;

    std_msgs::Float32MultiArray coordinate;
    Publisher coor_pub = nh.advertise<std_msgs::Float32MultiArray> ("visual_coordinate", 1);
    Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("visual_output", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

public:
    Stereo(){
        // string setting
        camera_L = "left";
        camera_R = "right";

        fu_L = 1766.56232;
        fv_L = 1766.80239;
        u0_L = 1038.99491;
        v0_L = 783.18243;
        kc_L << -0.00825394043969168, 0.0179401516569629, 4.99404147750032e-05, 0.00191928903162996, 0.00543957313843740, -0.0137634713444645, -3.37506702874865e-05, -0.00143562480046507;
        /*
        kc_L[0] = -0.00825394043969168;
        kc_L[1] = 0.0179401516569629;
        kc_L[2] = 4.99404147750032e-05;
        kc_L[3] = 0.00191928903162996;
        kc_L[4] = 0.00543957313843740;
        kc_L[5] = -0.0137634713444645;
        kc_L[6] = -3.37506702874865e-05;
        kc_L[7] = -0.00143562480046507;
        */

        fu_R = 1764.19837;
        fv_R = 1765.54600;
        u0_R = 1056.77639;
        v0_R = 773.98008;
        kc_R << -0.00771656075847792, 0.0111416719316138, 0.000739495171748185, 0.000840103654848698, 0.00584317345777879, -0.00874157051790497, -0.000384902445181700, -0.000593299662265151;
        /*
        kc_R[0] = -0.00771656075847792;
        kc_R[1] = 0.0111416719316138;
        kc_R[2] = 0.000739495171748185;
        kc_R[3] = 0.000840103654848698;
        kc_R[4] = 0.00584317345777879;
        kc_R[5] = -0.00874157051790497;
        kc_R[6] = -0.000384902445181700;
        kc_R[7] = -0.000593299662265151;
        */

        dis_Ix_L = -1;
        dis_Iy_L = -1;
        dis_Ix_R = -1;
        dis_Iy_R = -1;

        ID_L = 0;
        ID_R = 0;

        R_R2L << 0.9633, -0.1044, 0.2474, 0.1166, 0.9925, -0.0353, -0.2419, 0.0628, 0.9683;
        b_R2L << -840.38437, -115.52910, 232.14452;
        b_L2R << 879.1440, 12.3325, -20.9244;
        b_L2W << -699.620721, 450.703227, 2042.738938;
        R_W2L << 0.999942, 0.001900, -0.010572, -0.009106, -0.372230, -0.928096, -0.005698, 0.928139, -0.372191;
        compesation_value << -2.42, 5.1723, 30.0654;
        
        d = R_R2L*b_L2R;

        //x2 = 0;
        ID_L_past = -1;
        ID_R_past = -1;
        ID_L = 1;
        ID_R = 1;

        //y2 = 0;
        //z2 = 0;
        i = 0;

        cloud.width  = 100000;
        cloud.height = 1;
        cloud.points.resize(cloud.width * cloud.height);

        sub_left = nh.subscribe("ball_center_left", 1, &Stereo::callback_left, this);
        sub_right = nh.subscribe("ball_center_right", 1, &Stereo::callback_right, this);
    }

    void operator()(){
        //std::ofstream myfile;
        //myfile.open ("/home/lab606a/Documents/tmp.csv");
        while (ros::ok())
        {
            
            id_L = ID_L;
            id_R = ID_R;
            if ((id_L == id_R) && (ID_L_past != id_L) && (ID_R_past != id_R)){
                ID_L_past = id_L;
                ID_R_past = id_R;
                //start_ = ros::Time::now().toSec();
                if ((dis_Ix_L >= 0) && (dis_Iy_L >= 0) && (dis_Ix_R >= 0) && (dis_Iy_R >= 0)/* && (ID_L == ID_R)*/){
                    correction_img(camera_L, dis_Ix_L, dis_Iy_L, fu_L, fv_L, u0_L, v0_L, kc_L);
                    correction_img(camera_R, dis_Ix_R, dis_Iy_R, fu_R, fv_R, u0_R, v0_R, kc_R);

                    // calcuate k
                    k = ((R_R2L(0,0)*(Ix_L-u0_L)/fu_L) + (R_R2L(0,1)*(Iy_L-v0_L)/fv_L) + R_R2L(0,2)) - ((Ix_R-u0_R)/fu_R)*((R_R2L(2,0)*(Ix_L-u0_L)/fu_L) + (R_R2L(2,1)*(Iy_L-v0_L)/fv_L) + R_R2L(2,2));
                    //cout << "k = " << k << endl;
                    // calculate left ray vector
                    hz_L = (d(0) - (d(2)*(Ix_R-u0_R)/fu_R)) / k;
                    hx_L = hz_L*(Ix_L-u0_L)/fu_L;
                    hy_L = hz_L*(Iy_L-v0_L)/fv_L;

                    h_L << hx_L, hy_L, hz_L;

                    h_W = R_W2L.transpose() * (h_L-b_L2W) - compesation_value;

                    hx = h_W(0)/10;
                    hy = h_W(1)/10;
                    hz = h_W(2)/10;
                    //cout << hx << ", " << hy << ", " << hz << endl;

                    //y1 = int(h_W(1));
                    //z1 = int(h_W(2));
                    
                    if (hy < (-50)){
                        coordinate.data.push_back(0);
                        coordinate.data.push_back(0);
                        coordinate.data.push_back(0);
                        coordinate.data.push_back(0);

                        coor_pub.publish(coordinate);
                        coordinate.data.clear();

                        cloud.points.clear();
                        cloud.points.resize(cloud.width * cloud.height);
                        i = 0;

                        //myfile << "\n";
                    }
                    else{
                        //myfile << hx << "," << hy << "," << hz << ",";
                        coordinate.data.push_back(1);
                        coordinate.data.push_back(hx);
                        coordinate.data.push_back(hy);
                        coordinate.data.push_back(hz);

                        coor_pub.publish(coordinate);
                        coordinate.data.clear();

                        cloud.points[i].x = hx/100;
                        cloud.points[i].y = hy/100;
                        cloud.points[i].z = hz/100;
                        pcl::toROSMsg(cloud, output);
                        output.header.frame_id = "world";
                        pcl_pub.publish(output);
                    }
                    i += 1;
                    /*
                    if ((y1 != y2) && (z1 != z2)){
                        if (hy <= (-50)){
                            coordinate.data.push_back(0);
                            coordinate.data.push_back(0);
                            coordinate.data.push_back(0);
                            coordinate.data.push_back(0);

                            coor_pub.publish(coordinate);
                            coordinate.data.clear();

                            cloud.points.clear();
                            cloud.points.resize(cloud.width * cloud.height);
                            i = 0;
                            
                        }
                        else{
                            coordinate.data.push_back(1);
                            coordinate.data.push_back(hx);
                            coordinate.data.push_back(hy);
                            coordinate.data.push_back(hz);

                            coor_pub.publish(coordinate);
                            coordinate.data.clear();

                            cloud.points[i].x = hx;
                            cloud.points[i].y = hy;
                            cloud.points[i].z = hz;
                            pcl::toROSMsg(cloud, output);
                            output.header.frame_id = "map";
                            pcl_pub.publish(output);
                        }

                        //x2 = x1;
                        y2 = y1;
                        z2 = z1;
                        i = i+1;
                    }
                    */
                    //end_ = ros::Time::now().toSec();
                    //cout << (end_-start_) << endl;
                }
                
            }
            
        }
        //myfile.close();
    }


    void callback_left(const std_msgs::Int64MultiArray::ConstPtr& msg_left){
        ID_L = msg_left->data[0];
        dis_Ix_L = msg_left->data[1];
        dis_Iy_L = msg_left->data[2];
        contour_L = msg_left->data[3];
    }
    void callback_right(const std_msgs::Int64MultiArray::ConstPtr& msg_right){
        ID_R = msg_right->data[0];
        dis_Ix_R = msg_right->data[1];
        dis_Iy_R = msg_right->data[2];
        contour_R = msg_right->data[3];
    }

    void correction_img(string camera, int dis_Ix, int dis_Iy, double fu, double fv, double u0, double v0, Ref<RowVector8i> kc){
        double dis_hxz, dis_hyz, rd ,G, hxz, hyz;

        if (camera == "left"){
            // calculate distortion ray vector
            dis_hxz = (dis_Ix - u0) / fu;
            dis_hyz = (dis_Iy - v0) / fv;

            // calcuate correction parameter
            rd = sqrt(pow(dis_hxz,2)+pow(dis_hyz,2));
            G = 4*kc[4]*pow(rd,2) + 6*kc[5]*pow(rd,4) + 8*kc[6]*dis_hyz + 8*kc[7]*dis_hxz + 1;

            // calculate correction sight vector
            hxz = dis_hxz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hxz+2*kc[2]*dis_hxz*dis_hyz+kc[3]*(pow(rd,2)+2*pow(dis_hxz,2)));
            hyz = dis_hyz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hyz+kc[2]*(pow(rd,2)+2*pow(dis_hyz,2))+2*kc[3]*dis_hxz*dis_hyz);

            // calculate correction position
            Ix_L = u0 + fu*hxz;
            Iy_L = v0 + fv*hyz;
        }
        if (camera == "right"){
            // calculate distortion sight vector
            dis_hxz = (dis_Ix - u0) / fu;
            dis_hyz = (dis_Iy - v0) / fv;

            // calcuate correction parameter
            rd = sqrt(pow(dis_hxz,2)+pow(dis_hyz,2));
            G = 4*kc[4]*pow(rd,2) + 6*kc[5]*pow(rd,4) + 8*kc[6]*dis_hyz + 8*kc[7]*dis_hxz + 1;

            // calculate correction ray vector
            hxz = dis_hxz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hxz+2*kc[2]*dis_hxz*dis_hyz+kc[3]*(pow(rd,2)+2*pow(dis_hxz,2)));
            hyz = dis_hyz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hyz+kc[2]*(pow(rd,2)+2*pow(dis_hyz,2))+2*kc[3]*dis_hxz*dis_hyz);

            // calculate correction position
            Ix_R = u0 + fu*hxz;
            Iy_R = v0 + fv*hyz;
        }
    }

    /*
    void printttt(Eigen::Ref<RowVector3ii> M){
        cout << M[0] << M[1] << M[2] << endl;
    }
    */



};

int main(int argc, char **argv)
{
    init(argc, argv, "stereoooo");

    Stereo s;
    thread t(ref(s));

    spin();

    t.join();

    return 0;
}