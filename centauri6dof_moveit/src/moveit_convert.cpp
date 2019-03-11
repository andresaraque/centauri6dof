// cargo ros
#include "ros/ros.h"
// mensaje de valores de articulaciones mediante libreria jointState 
#include "sensor_msgs/JointState.h"
// ArmJointState se crea automaticamente en la libreria de arduino mediante ros_lib
#include "centauri6dof_moveit/ArmJointState.h"
#include "math.h"

// cargo de estados de articulaciones mediante el parametro arm_step
centauri6dof_moveit::ArmJointState arm_steps;
centauri6dof_moveit::ArmJointState total;
// micro pasos/revolución (usando 16tHz) por observación, para cada motor
int stepsPerRevolution[7] = {32000,16400,72000,3200,14400,3000,0}; 
int joint_status = 0;
double cur_angle[7];
int joint_step[7];
double prev_angle[7] = {0,0,0,0,0,0,0};
double init_angle[7] = {0,0,0,0,0,0,0};
double total_steps[7] = {0,0,0,0,0,0,0};
// contador para indicar si ya se tiene una posicion establecida
int count = 0;


//funcion para hacer llamdo de comandos por posicion 
void cmd_cb(const sensor_msgs::JointState& cmd_arm)
{
  if (count==0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
    prev_angle[3] = cmd_arm.position[3];
    prev_angle[4] = cmd_arm.position[4];
    prev_angle[5] = cmd_arm.position[5];
    prev_angle[6] = cmd_arm.position[6];

    init_angle[0] = cmd_arm.position[0];
    init_angle[1] = cmd_arm.position[1];
    init_angle[2] = cmd_arm.position[2];
    init_angle[3] = cmd_arm.position[3];
    init_angle[4] = cmd_arm.position[4];
    init_angle[5] = cmd_arm.position[5];
    init_angle[6] = cmd_arm.position[6];
  }

  //mostar en consola  que se reciben los parametros de posicion 
  ROS_INFO_STREAM("Recibido /move_group/fake_controller_joint_states");

// convertir parametros de pasos por revolucion a radianes 
  arm_steps.position1 = (int)((cmd_arm.position[0]-prev_angle[0])*stepsPerRevolution[0]/(2*M_PI));
  arm_steps.position2 = (int)((cmd_arm.position[1]-prev_angle[1])*stepsPerRevolution[1]/(2*M_PI));
  arm_steps.position3 = (int)((cmd_arm.position[2]-prev_angle[2])*stepsPerRevolution[2]/(2*M_PI));
  arm_steps.position4 = (int)((cmd_arm.position[3]-prev_angle[3])*stepsPerRevolution[3]/(2*M_PI));
  arm_steps.position5 = (int)((cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/(2*M_PI));
  arm_steps.position6 = (int)((cmd_arm.position[5]-prev_angle[5])*stepsPerRevolution[5]/(2*M_PI));
  arm_steps.position7 = (int)((cmd_arm.position[6]-prev_angle[6])*stepsPerRevolution[6]/(2*M_PI));

  //%d para imprimir los parametros de posicion 6 como test
  ROS_INFO_NAMED("test", "arm_steps.position6 #2: %d", arm_steps.position6);

// condicion si el contador es diferente de cero, actualiza las posiciones
  if (count!=0){
    prev_angle[0] = cmd_arm.position[0];
    prev_angle[1] = cmd_arm.position[1];
    prev_angle[2] = cmd_arm.position[2];
    prev_angle[3] = cmd_arm.position[3];
    prev_angle[4] = cmd_arm.position[4];
    prev_angle[5] = cmd_arm.position[5];
    prev_angle[6] = cmd_arm.position[6];
  }

//pasos totales tomados para llegar al objetivo
  total.position1 += arm_steps.position1;
  total.position2 += arm_steps.position2;
  total.position3 += arm_steps.position3;
  total.position4 += arm_steps.position4;
  total.position5 += arm_steps.position5;
  total.position6 += arm_steps.position6;

  ROS_INFO_NAMED("test", "total_steps[5]: %f, total: %d", total_steps[5], total.position6);
  ROS_INFO_NAMED("test", "arm_steps.position6 #3: %d", arm_steps.position6);

// consola informacion de que ya ha convetido los pasos 
  ROS_INFO_STREAM("Conversion hecha a /joint_steps");
  //empiza a correr los comando de void loop() de arduino 
  joint_status = 1;
  // contador en 1 para saber posicion previa de inicio 
  count=1;
}
//main fuction 
int main(int argc, char **argv)
{
  //incia paquete de moveit del robot 
  ros::init(argc, argv, "centauri6dof_moveit"); 
  // comunicacion serial con ros
  ros::NodeHandle nh;
  //mensaje de consola de incio de la funcion
  ROS_INFO_STREAM("Funcion principal");
  // subcripcion a los paquetes de las articulaciones del robot
  ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states",1000,cmd_cb);
  //publica las articulaciones 
  ros::Publisher pub = nh.advertise<centauri6dof_moveit::ArmJointState>("joint_steps",50);
// frecuencia a la que corre el loop 20hz
  ros::Rate loop_rate(50);
//Controlador SIGINT Instalado por roscpp para manejo de ctrl-C que suspende operaciones, ros::ok return false
  while (ros::ok())
  {
    if(joint_status==1)
    {
      joint_status = 0;
      //pub.publish(arm_steps);

      // publicar estados de articulaciones
      pub.publish(total);
      ROS_INFO_STREAM("Publicado en /joint_steps");
    }
    ros::spinOnce();
    // modo de reposo del loop 
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
