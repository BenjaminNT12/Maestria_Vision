#include <unistd.h> //librerias
#include <math.h>//librerias
#include <linux/joystick.h>//librerias
#include <fcntl.h>//librerias
#include <sys/stat.h>//librerias
#include <dirent.h>//librerias
#include <diagnostic_updater/diagnostic_updater.h>//librerias
#include <ros/ros.h>//librerias
#include <sensor_msgs/Joy.h>//librerias
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

///\brief Opens, reads from and publishes joystick events
class Joystick // Inicializamos clase Joystick
{

// funciones privadas
private: // seccion privada
  ros::NodeHandle nh_; // Creamos un objeto de la clase ros::NodeHandle llamado  nh_
  ros::NodeHandle nh_camera_bebop; // Creamos un objeto de la clase ros::NodeHandle llamado  nh_
  ros::NodeHandle nh_cmd_vel_bebop; // Creamos un objeto de la clase ros::NodeHandle llamado  nh_
  ros::NodeHandle nh_takeoff_bebop; // Creamos un objeto de la clase ros::NodeHandle llamado  nh_
  ros::NodeHandle nh_land_bebop; // Creamos un objeto de la clase ros::NodeHandle llamado  nh_
  bool open_; // Creamos un objeto de tipo boleano
  bool fly_active;
  bool control_active;
  bool once_fly;
  bool sticky_buttons_; // Creamos un objeto llamado sticky_buttons de tipo boleano
  bool default_trig_val_; // Creamos un objeto llmado default_trig_val de tipo boleano


  std::string joy_dev_; // creamos un objeto string de la clase std llamado joy_dev_
  std::string joy_dev_name_; // creamos un objeto string de la casle std llamado joy_dev_name_
  double deadzone_; // creamos un objeto de tipo double llamado deadzone_
  double autorepeat_rate_;  // creamos un objeto de tipo double llamado autorepeat_rate
  double coalesce_interval_; // creamos un objeto de tipo double llamado coalesce_interval_
  int event_count_; // creamos un objeto de tipo int llamado event_count_
  int pub_count_; // creamos un objeto de tipo int llamado pub_count_
  int queue_size;
  ros::Publisher pub_; // creamos un objeto de la clase ros de tipo Publisher llamado pub_
  ros::Publisher pub_bebop_camera; // creamos un objeto de la clase ros de tipo Publisher llamado pub_ /****** DEFINIR NUEVO OBJETO ros::Publisher objeto**************************************//
  ros::Publisher pub_bebop_cmd_vel;
  ros::Publisher pub_bebop_takeoff;
  ros::Publisher pub_bebop_land;
/*
  pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
  pub2 = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
  pub3 = rospy.Publisher('bebop/land', Empty, queue_size = 1)
*/

  double lastDiagTime_; // // creamos un objeto de tipo double llamado lastDiagTime_
  diagnostic_updater::Updater diagnostic_;  // creamos un objeto de la clase diagnostic_updater de tipo Updater llamado diagnostic_



  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) // creamos un metodo llamado diagnostics con argumento de la clase diagnostic_updater de tipo DiagnosticStatusWrapper puntero a stat
  {
    double now = ros::Time::now().toSec();
    double interval = now - lastDiagTime_;
    if (open_)
      stat.summary(0, "OK");
    else
      stat.summary(2, "Joystick not open.");

    stat.add("topic", pub_.getTopic());
    stat.add("device", joy_dev_);
    stat.add("device name", joy_dev_name_);
    stat.add("dead zone", deadzone_);
    stat.add("autorepeat rate (Hz)", autorepeat_rate_);
    stat.add("coalesce interval (s)", coalesce_interval_);
    stat.add("recent joystick event rate (Hz)", event_count_ / interval);
    stat.add("recent publication rate (Hz)", pub_count_ / interval);
    stat.add("subscribers", pub_.getNumSubscribers());
    stat.add("default trig val", default_trig_val_);
    stat.add("sticky buttons", sticky_buttons_);
    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = now;
  }

  std::string get_dev_by_joy_name(const std::string& joy_name)
  {
    const char path[] = "/dev/input";
    struct dirent *entry;
    struct stat stat_buf;

    DIR *dev_dir = opendir(path);
    if (dev_dir == NULL)
    {
      ROS_ERROR("Couldn't open %s. Error %i: %s.", path, errno, strerror(errno));
      return "";
    }

    while ((entry = readdir(dev_dir)) != NULL)
    {
      if (strncmp(entry->d_name, "js", 2) != 0)
        continue;
      std::string current_path = std::string(path) + "/" + entry->d_name;
      if (stat(current_path.c_str(), &stat_buf) == -1)
        continue;
      if (!S_ISCHR(stat_buf.st_mode))
        continue;

      int joy_fd = open(current_path.c_str(), O_RDONLY);
      if (joy_fd == -1)
        continue;

      char current_joy_name[128];
      if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)), current_joy_name) < 0)
        strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));

      close(joy_fd);

      ROS_INFO("Found joystick: %s (%s).", current_joy_name, current_path.c_str());

      if (strcmp(current_joy_name, joy_name.c_str()) == 0)
      {
          closedir(dev_dir);
          return current_path;
      }
    }

    closedir(dev_dir);
    return "";
  }

// TERMINA LA PARTE PUBLICA

// METODOS publicos
public:
  Joystick() : nh_(), diagnostic_(){} // constructor de la clase  --> ros::NodeHandle nh;  --> diagnostic_updater::Updater diagnostic_;

  double Interpolacion(double Val,double in1,double in2,double y1,double y2) { // agregar un metodo publico llamado Interpolacion

  	return (((Val - in1) / (in2 - in1)) * (y2 - y1) + y1);
  }

  int main(int argc, char **argv)
  {
    diagnostic_.add("Joystick Driver Status", this, &Joystick::diagnostics);
    diagnostic_.setHardwareID("none");
    geometry_msgs::Twist bebop_camera_msgs; /*********************DEFINIR EL NUEVO TIPO DE MENSAJE *******************************************************************/
    geometry_msgs::Twist bebop_cmd_vel_msgs;
    std_msgs::Empty bebop_takeoff_msgs;
    std_msgs::Empty bebop_land_msgs;
    fly_active = false;
    once_fly = false;
    control_active = false;
    double camera_val_axis_y = 0;
    double camera_val_axis_z = 0;
    // definicion de parametros
    ros::NodeHandle nh_param("~");
    pub_ = nh_.advertise<sensor_msgs::Joy>("bebop_fly_topic", 1);// nombre del topico
    nh_param.param<std::string>("dev", joy_dev_, "/dev/input/js0");
    nh_param.param<std::string>("dev_name", joy_dev_name_, "");
    nh_param.param<double>("deadzone", deadzone_, 0.05);
    nh_param.param<double>("autorepeat_rate", autorepeat_rate_, 50.0);
    nh_param.param<double>("coalesce_interval", coalesce_interval_, 0.001);
    nh_param.param<bool>("default_trig_val",default_trig_val_,false);
    nh_param.param<bool>("sticky_buttons", sticky_buttons_, false);

    ros::NodeHandle nh_camera_bebop_param("~");/**********************************DEFINIMOS EL NUEVO HANDLE ros::NodeHandle ******************************************************/
    pub_bebop_camera = nh_camera_bebop.advertise<geometry_msgs::Twist>("bebop/camera_control",1);/********************************Concatenamos el objeto NodeHandle********************************************************/

    ros::NodeHandle nh_cmd_vel_bebop_param("~");
    pub_bebop_cmd_vel = nh_cmd_vel_bebop.advertise<geometry_msgs::Twist>("bebop/cmd_vel",1);
    nh_cmd_vel_bebop_param.param<double>("autorepeat_rate", autorepeat_rate_, 50.0);

    ros::NodeHandle nh_takeoff_bebop_param("~");
    pub_bebop_takeoff = nh_takeoff_bebop.advertise<std_msgs::Empty>("bebop/takeoff",1,true);

    ros::NodeHandle nh_land_bebop_param("~");
    pub_bebop_land = nh_land_bebop.advertise<std_msgs::Empty>("bebop/land",1,true);

    // Checks on parameters //verificacion de parametros verifica que existen errores o informes a notificar
    if (!joy_dev_name_.empty())
    {
        std::string joy_dev_path = get_dev_by_joy_name(joy_dev_name_);
        if (joy_dev_path.empty())
            ROS_ERROR("Couldn't find a joystick with name %s. Falling back to default device.", joy_dev_name_.c_str());
        else
        {
            ROS_INFO("Using %s as joystick device.", joy_dev_path.c_str());
            joy_dev_ = joy_dev_path;
        }
    }

    if (autorepeat_rate_ > 1 / coalesce_interval_)
      ROS_WARN("joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f Hz) does not make sense. Timing behavior is not well defined.", autorepeat_rate_, 1/coalesce_interval_);

    if (deadzone_ >= 1)
    {
      ROS_WARN("joy_node: deadzone greater than 1 was requested. The semantics of deadzone have changed. It is now related to the range [-1:1] instead of [-32767:32767]. For now I am dividing your deadzone by 32767, but this behavior is deprecated so you need to update your launch file.");
      deadzone_ /= 32767;
    }

    if (deadzone_ > 0.9)
    {
      ROS_WARN("joy_node: deadzone (%f) greater than 0.9, setting it to 0.9", deadzone_);
      deadzone_ = 0.9;
    }

    if (deadzone_ < 0)
    {
      ROS_WARN("joy_node: deadzone_ (%f) less than 0, setting to 0.", deadzone_);
      deadzone_ = 0;
    }

    if (autorepeat_rate_ < 0)
    {
      ROS_WARN("joy_node: autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate_);
      autorepeat_rate_ = 0;
    }

    if (coalesce_interval_ < 0)
    {
      ROS_WARN("joy_node: coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval_);
      coalesce_interval_ = 0;
    }

    // convertimos los parametros
    double autorepeat_interval = 1 / autorepeat_rate_;
    double scale = -1. / (1. - deadzone_) / 32767.;
    double unscaled_deadzone = 32767. * deadzone_;

    js_event event; // crea un objeto de tipo js_event
    struct timeval tv;
    fd_set set;
    int joy_fd;
    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = ros::Time::now().toSec();
/******************************************************************************/

    while (nh_.ok())
    {
      open_ = false;
      diagnostic_.force_update();
      bool first_fault = true;
      while (true)
      {
        ros::spinOnce(); // solo ejecuta un thread
        if (!nh_.ok())
          goto cleanup;
        joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        if (joy_fd != -1)
        {
          close(joy_fd);
          joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        }
        if (joy_fd != -1)
          break;
        if (first_fault)
        {
          ROS_ERROR("Couldn't open joystick %s. Will retry every second.", joy_dev_.c_str());
          first_fault = false;
        }
        sleep(1.0);
        diagnostic_.update();
      } // TERMINAL WHILE (TRUE)
      ROS_INFO("Opened joystick: %s. deadzone_: %f.", joy_dev_.c_str(), deadzone_);
      open_ = true;
      diagnostic_.force_update();

      bool tv_set = false;
      bool publication_pending = false;
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      sensor_msgs::Joy joy_msg;
      double val;
      sensor_msgs::Joy last_published_joy_msg;
      sensor_msgs::Joy sticky_buttons_joy_msg;
      while (nh_.ok())
      {
        ros::spinOnce();

        bool publish_now = false;
        bool publish_soon = false;
        FD_ZERO(&set);
        FD_SET(joy_fd, &set);

        int select_out = select(joy_fd+1, &set, NULL, NULL, &tv);
        if (select_out == -1)
        {
          tv.tv_sec = 0;
          tv.tv_usec = 0;
          continue;
        }
        if (FD_ISSET(joy_fd, &set))
        {
          if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
            break;

          joy_msg.header.stamp = ros::Time().now();
          event_count_++;
          switch(event.type)
          {
          case JS_EVENT_BUTTON: //  JS_EVENT_BUTTON = 0x01

          case JS_EVENT_BUTTON | JS_EVENT_INIT: // JS_EVENT_BUTTON = 0x01 or JS_EVENT_INIT = 0x80
            if(event.number >= joy_msg.buttons.size())
            {
              int old_size = joy_msg.buttons.size();
              joy_msg.buttons.resize(event.number+1);
              last_published_joy_msg.buttons.resize(event.number+1);
              sticky_buttons_joy_msg.buttons.resize(event.number+1);
              for(unsigned int i=old_size;i<joy_msg.buttons.size();i++){
                joy_msg.buttons[i] = 0.0;
                last_published_joy_msg.buttons[i] = 0.0;
                sticky_buttons_joy_msg.buttons[i] = 0.0;
              }
            }
            joy_msg.buttons[event.number] = (event.value ? 1 : 0);


            if( joy_msg.buttons[7] == 1 && control_active == false){
              control_active = true;
            }else if( joy_msg.buttons[7] == 1 && control_active == true){
              control_active = false;
              ROS_WARN("control_active, valor = %b", control_active);
            }

            if( joy_msg.buttons[8] == 1 && fly_active == false){
              fly_active = true;
            }else if( joy_msg.buttons[8] == 1 && fly_active == true){
              fly_active = false;
            }
            if (!(event.type & JS_EVENT_INIT))
              publish_now = true;
            else
              publish_soon = true;
            break;
          case JS_EVENT_AXIS: // JS_EVENT_AXIS = 0x02
// comprueba que este iniciado y que hay sucedido el evento de eje
          case JS_EVENT_AXIS | JS_EVENT_INIT: // JS_EVENT_AXIS = 0x02 or  JS_EVENT_INIT = 0x80
            val = event.value; // el valor de los ejes del control
            if(event.number >= joy_msg.axes.size())
            {
              int old_size = joy_msg.axes.size();
              joy_msg.axes.resize(event.number+1);
              last_published_joy_msg.axes.resize(event.number+1);
              sticky_buttons_joy_msg.axes.resize(event.number+1);
              for(unsigned int i=old_size;i<joy_msg.axes.size();i++){
                joy_msg.axes[i] = 0.0;
                last_published_joy_msg.axes[i] = 0.0;
                sticky_buttons_joy_msg.axes[i] = 0.0;
              }
            }
            if(default_trig_val_){
              if (val > unscaled_deadzone)
                val -= unscaled_deadzone;
              else if (val < -unscaled_deadzone)
                val += unscaled_deadzone;
              else
                val = 0;
              joy_msg.axes[event.number] = val * scale;/*******************************************/
              publish_soon = true;
              break;
            }
            else
            {
              if (!(event.type & JS_EVENT_INIT)){ // entra porque se aplica una and a nivel de bits resultado 0 Cero
                val = event.value;
                if(val > unscaled_deadzone)
                  val -= unscaled_deadzone;
                else if(val < -unscaled_deadzone)
                  val += unscaled_deadzone;
                else
                  val=0;

                  joy_msg.axes[event.number] = val * scale;// Estos valores son los que se envian

                bebop_cmd_vel_msgs.linear.x = joy_msg.axes[3];
                bebop_cmd_vel_msgs.linear.y = joy_msg.axes[2];
                bebop_cmd_vel_msgs.linear.z = joy_msg.axes[1];

                bebop_cmd_vel_msgs.angular.x = 0.0;
                bebop_cmd_vel_msgs.angular.y += 0.00001;
                bebop_cmd_vel_msgs.angular.z = joy_msg.axes[0];

                bebop_camera_msgs.angular.y =  Interpolacion(joy_msg.axes[7],-1,1,-80,80); // lleno los valores del buffer para el envio de los valores de los ejes
                bebop_camera_msgs.angular.z = Interpolacion(joy_msg.axes[6],-1,1,35,-35); // lleno los valores del buffer para el envio de los valores de los ejes
                // ROS_WARN("bebop_camera_msgs.angular.y, valor = %d ,bebop_camera_msgs.angular.z, valor = %d", bebop_camera_msgs.angular.y, bebop_camera_msgs.angular.z);
                //joy_msg.axes[event.number]= val;/*******************************************/
              }

              publish_soon = true;
              break;
              default:
              ROS_WARN("joy_node: Unknown event type. Please file a ticket. time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
              break;
            }
          }// termina el switch case
        }
        else if (tv_set)
        {
          joy_msg.header.stamp = ros::Time().now();
          publish_now = true;
        }
        if (publish_now) {
          if (sticky_buttons_ == true) {
            for (size_t i = 0; i < joy_msg.buttons.size(); i++) {  // llena el buffer de todos botones una y otra vez
              if (joy_msg.buttons[i] == 1 && last_published_joy_msg.buttons[i] == 0) {
                sticky_buttons_joy_msg.buttons[i] = sticky_buttons_joy_msg.buttons[i] ? 0 : 1; // llena el buffer de comunicacion
              } else {
              }
            }
            last_published_joy_msg = joy_msg; // actualiza el valor del ultimo valor presionado
            sticky_buttons_joy_msg.header.stamp.nsec = joy_msg.header.stamp.nsec;
            sticky_buttons_joy_msg.header.stamp.sec  = joy_msg.header.stamp.sec;
            sticky_buttons_joy_msg.header.frame_id   = joy_msg.header.frame_id;
            for(size_t i=0; i < joy_msg.axes.size(); i++){ // llena el buffer de con los valores de posicion del los ejes
              sticky_buttons_joy_msg.axes[i] = joy_msg.axes[i];
            }
            //pub_bebop_camera.publish(bebop_camera_msgs); // hago la publicacion del mensaje para el envio // REVISAR ?????????????
            pub_.publish(sticky_buttons_joy_msg); // publica todo el mensaje
          } else {
/// EN ESTA PARTE SE ENVIAN TODOS LOS DATOS
            joy_msg.header.stamp = ros::Time().now();
            //pub_.publish(sticky_buttons_joy_msg);
            if(fly_active == true && once_fly == false){
              once_fly = true;
              pub_bebop_takeoff.publish(bebop_takeoff_msgs);
            }
            if(fly_active == false && once_fly == true){
              once_fly = false;
              pub_bebop_land.publish(bebop_land_msgs);
            }

            if(control_active == true){
              pub_bebop_cmd_vel.publish(bebop_cmd_vel_msgs);
              // pub_bebop_camera.publish(bebop_camera_msgs);
            }
            pub_.publish(joy_msg); // envia el buffer de envio
          }
          publish_now = false;
          tv_set = false;
          publication_pending = false;
          publish_soon = false;
          pub_count_++;
        }
        if (!publication_pending && publish_soon)
        {
          tv.tv_sec = trunc(coalesce_interval_);
          tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
          publication_pending = true;
          tv_set = true;
        }
        if (!tv_set && autorepeat_rate_ > 0)
        {
          tv.tv_sec = trunc(autorepeat_interval);
          tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
          tv_set = true;
        }

        if (!tv_set)
        {
          tv.tv_sec = 1;
          tv.tv_usec = 0;
        }

        diagnostic_.update();
      }
      close(joy_fd);
      ros::spinOnce();
      if (nh_.ok())
        ROS_ERROR("Connection to joystick device lost unexpectedly. Will reopen.");
    }

  cleanup:
     ROS_INFO("joy_node shut down.");

    return 0;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_node");
  Joystick j;
  return j.main(argc, argv);
}
/**

l girar con manecillas del relog
j en contra
i-I frente
L = derecha sin rotar
J = izquierda sin rotar
I = frente
; = atras
t = subir
b = bajo
**/
