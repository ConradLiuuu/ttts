#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <iostream>
#include <math.h>
#include <thread>
using namespace std;
using namespace ros;

string camera_L = "left", camera_R = "right";
// left camera intrinsic parameter
double fu_L = 1765.39009, fv_L = 1764.93199;  // focal length
double u0_L = 1050.18724, v0_L = 776.90152;  // principal point
double kc_L[8] = {-0.00118917804407902, -0.000698693814115858, 0.00101005240597130, 2.99932211560579e-05, 0.000970791131425397, 0.000409044730994512, -0.000785103677409109, -2.03162761430176e-05};
// right camera intrinsic parameter
double fu_R = 1762.49155, fv_R = 1761.41036; // focal length
double u0_R = 1067.43219, v0_R = 770.87706; // principal point
double kc_R[8] = {0.00413435776050331, -0.0249585312796035, 0.00143012534004785, -0.00126956638842365, -0.00344738521837745, 0.0131320482232938, -0.000890878853209730, 0.000967412976435239};

//double a = 0,b = 0, c = 0, dd = 0;
int dis_Ix_L = -1, dis_Iy_L = -1, dis_Ix_R = -1, dis_Iy_R = -1;
double Ix_L, Iy_L, Ix_R, Iy_R;
//int ID_L, ID_R;
int ID_L = 0, ID_R = 0;
int id_L, id_R;
bool isDone = true;
int bound = -26;

class Sub_ball_center
{
private:
  NodeHandle nh;
  Subscriber sub_left, sub_right;

public:
  Sub_ball_center(){
    sub_left = nh.subscribe("ball_center_left", 1, &Sub_ball_center::callback_left, this);
    sub_right = nh.subscribe("ball_center_right", 1, &Sub_ball_center::callback_right, this);
  }

  void operator()(){
    sub_left = nh.subscribe("ball_center_left", 1, &Sub_ball_center::callback_left, this);
    sub_right = nh.subscribe("ball_center_right", 1, &Sub_ball_center::callback_right, this);
    ros::spin();
  }

  void callback_left(const std_msgs::Int64MultiArray::ConstPtr& msg_left)
  {
    if ((msg_left->data[1] >= 0) && (msg_left->data[2] >= 0)){
      ID_L = msg_left->data[0];
      dis_Ix_L = msg_left->data[1];
      dis_Iy_L = msg_left->data[2];
      //cout << "ID_L = " << ID_L << endl;
      //isDone = false;
    }
    else {
      ID_L = 0;
      dis_Ix_L = -1;
      dis_Iy_L = -1;
    }

    //cout << "dis_Ix_L = " << dis_Ix_L << endl;
    //cout << "dis_Iy_L = " << dis_Iy_L << endl;
  }

  void callback_right(const std_msgs::Int64MultiArray::ConstPtr& msg_right)
  {
    if ((msg_right->data[1] >= 0) && (msg_right->data[2] >= 0)){
      ID_R = msg_right->data[0];
      dis_Ix_R = msg_right->data[1];
      dis_Iy_R = msg_right->data[2];
      //cout << "ID_R = " << ID_R << endl;
      //isDone = false;
    }
    else{
      ID_R = -1;
      dis_Ix_R = -1;
      dis_Iy_R = -1;
    }

    //cout << "dis_Ix_R = " << dis_Ix_R << endl;
    //cout << "dis_Iy_R = " << dis_Iy_R << endl;
  }
};

void correction_img(string camera, int dis_Ix, int dis_Iy, double fu, double fv, double u0, double v0, double kc[8])
{
  double dis_hxz, dis_hyz, rd ,G, hxz, hyz;

  if (camera == "left"){
    // calculate distortion ray vector
    dis_hxz = (dis_Ix - u0) / fu;
    dis_hyz = (dis_Iy - v0) / fv;
    //cout << "distortion hxz_L = " << dis_hxz << endl;
    //cout << "distortion hyz_L = " << dis_hyz << endl;


    // calcuate correction parameter
    rd = sqrt(pow(dis_hxz,2)+pow(dis_hyz,2));
    G = 4*kc[4]*pow(rd,2) + 6*kc[5]*pow(rd,4) + 8*kc[6]*dis_hyz + 8*kc[7]*dis_hxz + 1;

    // calculate correction sight vector
    hxz = dis_hxz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hxz+2*kc[2]*dis_hxz*dis_hyz+kc[3]*(pow(rd,2)+2*pow(dis_hxz,2)));
    hyz = dis_hyz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hyz+kc[2]*(pow(rd,2)+2*pow(dis_hyz,2))+2*kc[3]*dis_hxz*dis_hyz);

    // calculate correction position
    Ix_L = u0 + fu*hxz;
    Iy_L = v0 + fv*hyz;
    //cout << "correct Ix_L Iy_L = [" << Ix_L << "," << Iy_L << "]" << endl;
    //cout << "Ix_L by func = " << Ix_L << endl;
    //cout << "Iy_L by func= " << Iy_L << endl;

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
    //cout << "correct Ix_R Iy_R = [" << Ix_R << "," << Iy_R << "]" << endl;
    //cout << "Ix_R by func = " << Ix_R << endl;
    //cout << "Iy_R by func= " << Iy_R << endl;

  }

}

void tt(){

  double R_R2L[3][3] = {{0.9984, -0.0559, 0.0008},{0.0559, 0.9984, -0.0071},{-0.0004, 0.0071, 0.9999}}; // matlab given
  double R_L2R[3][3] = {{R_R2L[0][0], R_R2L[1][0], R_R2L[2][0]},{R_R2L[0][1], R_R2L[1][1], R_R2L[2][1]},{R_R2L[0][2], R_R2L[1][2], R_R2L[2][2]}};

  double b_R2L[3] = {-311.77753, -16.36516, -6.95561}; // matlab given
  double b_L2R[3] = {312.2022, -1.0501, 7.0778}; // -R_L2R * b_R2L

  double d[3];
  d[0] = (R_R2L[0][0]*b_L2R[0]) + (R_R2L[0][1]*b_L2R[1]) + (R_R2L[0][2]*b_L2R[2]);
  d[1] = (R_R2L[1][0]*b_L2R[0]) + (R_R2L[1][1]*b_L2R[1]) + (R_R2L[1][2]*b_L2R[2]);
  d[2] = (R_R2L[2][0]*b_L2R[0]) + (R_R2L[2][1]*b_L2R[1]) + (R_R2L[2][2]*b_L2R[2]);

  double b_L2W[3] = {-470.261573, 644.159731, 1984.264350};
  double R_L2W[3][3] = {{0.999969, -0.005972, 0.005185}, {0.000430, -0.613575, -0.789637}, {0.007897, 0.789614, -0.613553}};
  double R_W2L[3][3] = {{R_L2W[0][0], R_L2W[1][0], R_L2W[2][0]},{R_L2W[0][1], R_L2W[1][1], R_L2W[2][1]},{R_L2W[0][2], R_L2W[1][2], R_L2W[2][2]}};

  double hx_L, hy_L, hz_L;

  double hx_W, hy_W, hz_W;
  double dif_L2W[3] = {0};

  double y1;
  double y2 = 0;
  int diff = 0;

  while (ros::ok()) {
    //ROS_INFO("dis_Ix_L = %f, dis_Iy_L = %f \n", a,b);
    //ROS_INFO("dis_Ix_R = %f, dis_Iy_R = %f \n", c,dd);
    //cout << dis_Ix_L << "," << dis_Iy_L << "|" << dis_Ix_R << "," << dis_Iy_R << endl;
    //cout << "ID_L = " << ID_L  << ", ID_R = " << ID_R << endl;

    //diff = ID_L - ID_R;
    id_L = ID_L;
    id_R = ID_R;

    if (id_L == id_R){
      if ((dis_Ix_L >= 0) && (dis_Iy_L >= 0) && (dis_Ix_R >= 0) && (dis_Iy_R >= 0) && (ID_L == ID_R)){
        correction_img(camera_L, dis_Ix_L, dis_Iy_L, fu_L, fv_L, u0_L, v0_L, kc_L);
        correction_img(camera_R, dis_Ix_R, dis_Iy_R, fu_R, fv_R, u0_R, v0_R, kc_R);

        // calcuate k
        double k = ((R_R2L[0][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[0][1]*(Iy_L-v0_L)/fv_L) + R_R2L[0][2]) - ((Ix_R-u0_R)/fu_R)*((R_R2L[2][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[2][1]*(Iy_L-v0_L)/fv_L) + R_R2L[2][2]);

        // calculate left ray vector
        hz_L = (d[0] - (d[2]*(Ix_R-u0_R)/fu_R)) / k;
        hx_L = hz_L*(Ix_L-u0_L)/fu_L;
        hy_L = hz_L*(Iy_L-v0_L)/fv_L;

        dif_L2W[0] = hx_L-b_L2W[0];
        dif_L2W[1] = hy_L-b_L2W[1];
        dif_L2W[2] = hz_L-b_L2W[2];

        hx_W = R_W2L[0][0]*dif_L2W[0] + R_W2L[0][1]*dif_L2W[1] + R_W2L[0][2]*dif_L2W[2];
        hy_W = R_W2L[1][0]*dif_L2W[0] + R_W2L[1][1]*dif_L2W[1] + R_W2L[1][2]*dif_L2W[2];
        hz_W = R_W2L[2][0]*dif_L2W[0] + R_W2L[2][1]*dif_L2W[1] + R_W2L[2][2]*dif_L2W[2];
        //cout << "object position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;

        hx_W = R_W2L[0][0] * dif_L2W[0] + R_W2L[0][1] * dif_L2W[1] + R_W2L[0][2] * dif_L2W[2]/* - (7.725139)*/;
        hy_W = R_W2L[1][0] * dif_L2W[0] + R_W2L[1][1] * dif_L2W[1] + R_W2L[1][2] * dif_L2W[2]/* - (1.153236)*/;
        hz_W = R_W2L[2][0] * dif_L2W[0] + R_W2L[2][1] * dif_L2W[1] + R_W2L[2][2] * dif_L2W[2] - (11.506209) + 16;

        y1 = hy_W;
        //diff = y1-y2;
        if ((y1 != y2) && (ID_L == ID_R )){
          //ROS_INFO("ID_L = %d, ID_R = %d", ID_L, ID_R);
          cout << "ID_L = " << ID_L  << ", ID_R = " << ID_R << endl;;
          ROS_INFO("correction position = [%f, %f, %f]", hx_W / 10, hy_W / 10,hz_W / 10);
          //ROS_INFO("correction position = [%f, %f, %f]", hx_W, hy_W, hz_W);
          //cout << "correction position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;
          y2 = y1;
        }
        isDone = true;
      }

    }

/*
    if ((dis_Ix_L >= 0) && (dis_Iy_L >= 0) && (dis_Ix_R >= 0) && (dis_Iy_R >= 0)){
      correction_img(camera_L, dis_Ix_L, dis_Iy_L, fu_L, fv_L, u0_L, v0_L, kc_L);
      correction_img(camera_R, dis_Ix_R, dis_Iy_R, fu_R, fv_R, u0_R, v0_R, kc_R);

      // calcuate k
      double k = ((R_R2L[0][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[0][1]*(Iy_L-v0_L)/fv_L) + R_R2L[0][2]) - ((Ix_R-u0_R)/fu_R)*((R_R2L[2][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[2][1]*(Iy_L-v0_L)/fv_L) + R_R2L[2][2]);

      // calculate left ray vector
      hz_L = (d[0] - (d[2]*(Ix_R-u0_R)/fu_R)) / k;
      hx_L = hz_L*(Ix_L-u0_L)/fu_L;
      hy_L = hz_L*(Iy_L-v0_L)/fv_L;

      dif_L2W[0] = hx_L-b_L2W[0];
      dif_L2W[1] = hy_L-b_L2W[1];
      dif_L2W[2] = hz_L-b_L2W[2];

      hx_W = R_W2L[0][0]*dif_L2W[0] + R_W2L[0][1]*dif_L2W[1] + R_W2L[0][2]*dif_L2W[2];
      hy_W = R_W2L[1][0]*dif_L2W[0] + R_W2L[1][1]*dif_L2W[1] + R_W2L[1][2]*dif_L2W[2];
      hz_W = R_W2L[2][0]*dif_L2W[0] + R_W2L[2][1]*dif_L2W[1] + R_W2L[2][2]*dif_L2W[2];
      //cout << "object position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;

      hx_W = R_W2L[0][0] * dif_L2W[0] + R_W2L[0][1] * dif_L2W[1] + R_W2L[0][2] * dif_L2W[2] - 17.6031;
      hy_W = R_W2L[1][0] * dif_L2W[0] + R_W2L[1][1] * dif_L2W[1] + R_W2L[1][2] * dif_L2W[2] - (-23.1113);
      hz_W = R_W2L[2][0] * dif_L2W[0] + R_W2L[2][1] * dif_L2W[1] + R_W2L[2][2] * dif_L2W[2] - 18.5448 + 16;

      y1 = hy_W;
      //diff = y1-y2;
      if (y1 != y2){
        cout << "ID_L = " << ID_L  << ", ID_R = " << ID_R << endl;;
        cout << "correction position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;
        y2 = y1;
      }

    }
    */
    /*else{
      cout << "not found ball" << endl;
    }*/

  }

}

int main(int argc, char** argv)
{
  init(argc, argv, "stereo_oop");
  //Stereo sterro;
  Sub_ball_center sub;

  thread t1(ref(sub));
  thread t2(tt);

  t2.join();
  t1.join();


  return 0;
}
