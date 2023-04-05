// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <iostream>
#include <chrono>

#include <arc/ArcTypes.h>
#include <arc/Robots/LBRIiwa/LBRIiwa.h>

/*** performance test results ***/
// simulation duration: 1000s
// without rendering: 1997ms

using RobotType = arc::Robots::Iiwa::LBRIiwa;
using RobotModel = arc::Robots::Iiwa::LBRIiwaModel;
using TrajectoryServer = arc::Robots::Iiwa::LBRIiwaTrajectoryServer;
using SilServer = arc::Robots::Iiwa::LBRIiwaSilServer;
using JointVector = RobotType::JointVector;
using JointArray = RobotType::JointArray;

using namespace arc;

bool iniOK = false;
bool hanging = false;
JointVector joint_init_pos;
int server_port = arc::Robots::Iiwa::SERVER_PORT;
int client_port = arc::Robots::Iiwa::CLIENT_PORT;

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right)
  {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

constexpr mjtNum DURATION = 1000.0;

// main function
int main(int argc, const char **argv)
{
  // check command-line arguments
  if (argc != 2)
  {
    std::printf(" USAGE:  basic modelfile\n");
    return 0;
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(argv[1]) > 4 && !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb"))
  {
    m = mj_loadModel(argv[1], 0);
  }
  else
  {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m)
  {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // // chrono high precision timer now
  auto start = std::chrono::high_resolution_clock::now();

  // while (d->time < DURATION)
  // {
  //   mj_step(m, d);
  //   // std::cout << "time-> " << d->time << std::endl;
  // }

  // auto end = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  // // cast duration to milliseconds
  // auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  // std::cout << "duration: " << duration_ms.count() << " milliseconds" << std::endl;

  /*** setup arc ***/
  joint_init_pos = JointVector::Zero();
  double Ts = m->opt.timestep;

  using namespace arc::Robots;

  // set controller parameter
  Iiwa::IiwaContrParam contr_param(true); // true..sim, false..lab
  JointArray f_c_slow_norm = contr_param.f_c_slow * (2 * Ts);
  JointArray f_c_fast_norm = contr_param.f_c_fast * (2 * Ts);

  auto arc_model_ptr = std::make_unique<Iiwa::LBRIiwaModel>(f_c_slow_norm, Ts);
  Iiwa::JointCTParameter js_param(contr_param);
  Iiwa::CartesianCTParameter ts_param(contr_param);
  Iiwa::SingularPerturbationParameter sp_param(contr_param.K_sp, contr_param.D_sp, f_c_fast_norm);
  Iiwa::JointVector B_fc = arc_model_ptr->get_B();
  Iiwa::FrictionCompensationParameter fc_param(contr_param.L_fc, B_fc, f_c_fast_norm);
  Iiwa::GravityCompParameter gc_param(contr_param.D_gc);

  // create instances
  arc::log::start_logging(arc::log::Level::Info);

  arc::log::write_info("Init LBRIiwa Controller");
  auto arc_contr_ptr = std::make_shared<Iiwa::LBRIiwa>(*arc_model_ptr, Ts, js_param, ts_param, sp_param, fc_param, gc_param, hanging);

  // Initialize controller
  arc_contr_ptr->set_singular_perturbation_state(false);
  arc_contr_ptr->set_friction_compensation_state(false);

  double m_ee = 0;
  double cog_ee_x = 0;
  double cog_ee_y = 0;
  double cog_ee_z = 0;
  arc_contr_ptr->init(m_ee, cog_ee_x, cog_ee_y, cog_ee_z);

  arc::log::write_info("Init TrajectoryServer");
  // com_server = std::make_unique<TrajectoryServer>(arc_contr_ptr, server_port, client_port);
  arc::log::write_info("Finished loading ARC Plugin.");



  JointVector q_act = Eigen::Map<JointVector>(d->qpos);
  double T_traj = 1;
  arc_contr_ptr->start(d->time, q_act, joint_init_pos, T_traj);

  // JointVector q_act;
  JointVector tau_sens_act = JointVector::Zero();
  JointVector tau_motor_act = JointVector::Zero();
  JointVector tau_set;



  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window))
  {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0)
    {
      mj_step(m, d);

      // copy qpos to q_act
      // for(int i=0;i<q_act.size();i++)
      // {
      //   q_act[i] = d->qpos[i];
      // }
      std::cout<<"entering update\r"<<std::endl;
      tau_set = arc_contr_ptr->update(0, q_act, tau_sens_act, tau_motor_act, false);
      tau_sens_act = tau_set;
      // for(int i=0;i<tau_set.size();i++)
      // {
      //   d->ctrl[i] = tau_set[i];
      // }

    }

    // std::cout << "d->time: " << d->time << "\r"<< std::flush;
    if (d->time > DURATION)
    {
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      // cast duration to milliseconds
      auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
      std::cout << "duration: " << duration_ms.count() << " milliseconds" << std::endl;
      break;
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
