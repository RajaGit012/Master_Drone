#! /usr/bin/env python

import numpy as np
import rospy
import actionlib
import dynamic_reconfigure.client
import spc_uav_comm.msg
import nav_msgs.msg
import tf2_ros
import tf.transformations
from learning_gui import LearningGUI

# Changed to be loaded from ROS Parameter Server
#END_EFFECTOR_POS = [0.7, 0, 0]
#FAIL_REWARD = -100


class LearningClient:
    def __init__(self, action_server='learning', dyn_reconfig_server='offboard_controller/motion'):
        rospy.init_node('learning_client_py')
        self.action_client = actionlib.SimpleActionClient(action_server, spc_uav_comm.msg.LearnParamAction)
        self.action_client.wait_for_server()
        self.action_running = self.task_started = False

        self.dyn_reconfig_client = dynamic_reconfigure.client.Client(dyn_reconfig_server)
        self.current_config = self.dyn_reconfig_client.get_configuration()
        self.default_config = self.current_config.copy()
        self.available_vars = list(self.default_config.keys())
        self.available_vars.remove('groups')
        self.available_vars.sort()
        self.controlled_vars = dict()
        self.fixed_vars = dict()
        self.linked_vars = dict()
        self.links = dict()

        self.set_point_sub = rospy.Subscriber("gui_setpoint_odometry", nav_msgs.msg.Odometry, self._set_point_CB)

        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)

        self.action_settings = {'depth':0.2, 'radius':0.5, 'speed':0.5, 'atype':0}

        # Loading Learning Parameters from ROS Parameter Server
        self.failure_reward = rospy.get_param('~failure_reward', -100)
        self.end_effector_pos = [rospy.get_param('~end_effector_pos', 0.7), 0.0, 0.0]
        self.local_pos_frame_id = rospy.get_param('~LocalPosFrameId', 'world')
        self.local_pos_child_frame_id = rospy.get_param('~LocalPosChildFrameId', 'base_link')

    def set_action_settings(self, **settings):
        for setting in settings:
            self.action_settings[setting] = settings[setting]

    def _get_local_pos(self):
        try:
            local_position = self.tfbuffer.lookup_transform(self.local_pos_frame_id,  self.local_pos_child_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        if 'local_position' in locals():
            if rospy.Time.now().to_sec() - local_position.header.stamp.to_sec() < 1.0:
                self.current_orient = [local_position.transform.rotation.x,
                                       local_position.transform.rotation.y,
                                       local_position.transform.rotation.z,
                                       local_position.transform.rotation.w]
                self.current_pos = np.matmul(tf.transformations.quaternion_matrix(
                                             self.current_orient)[0:3, 0:3],
                                             self.end_effector_pos) + np.array(
                                             [local_position.transform.translation.x,
                                              local_position.transform.translation.y,
                                              local_position.transform.translation.z])
                return True

            else:
                rospy.logerr("[" + str(local_position.header.stamp.to_sec())
                            + "]: Transform " + "world" + "->" + "base_link" + " timed out.")
        else:
            rospy.logerr("[" + str(rospy.Time.now().to_sec()) + "]: Transform "
                        + "world" + "->" + "base_link" + " does not exist.")

        return False

    def _set_point_CB(self, msg):
        if self.task_started and self._get_local_pos():
            self.set_pos = np.concatenate((self.set_pos,
                                           np.array([msg.pose.pose.position.x,
                                                     msg.pose.pose.position.y,
                                                     msg.pose.pose.position.z])
                                           .reshape((1, 3))))
            self.set_rot = np.concatenate((self.set_rot,
                                           np.array([msg.pose.pose.orientation.x,
                                                     msg.pose.pose.orientation.y,
                                                     msg.pose.pose.orientation.z,
                                                     msg.pose.pose.orientation.w])
                                           .reshape((1, 4))))

            self.real_pos = np.concatenate((self.real_pos, self.current_pos.reshape((1, 3))))
            self.real_rot = np.concatenate((self.real_rot, np.array(self.current_orient).reshape((1,4))))


    def _action_feedback_CB(self, feedback):
        # set parameters
        self.update_dyn_reconfig()
        self.task_started = feedback.started
        if self.task_started:
            self.set_pos = np.empty((0, 3))
            self.real_pos = np.empty((0, 3))
            self.set_rot = np.empty((0, 4))
            self.real_rot = np.empty((0, 4))
        rospy.loginfo("Learning predetermined task was initialized.")

    def _action_done_CB(self, status, result):
        #import pickle
        #with open('traj.pkl', 'wb') as f:
        #     pickle.dump((self.set_pos, self.real_pos), f)

        self.action_running = self.task_started = False

        if result:
            if result.finished:
                rospy.loginfo("Learning predetermined task was completed successfully.")
                rospy.loginfo("number of samples: " + str(self.real_pos.shape[0]))
                return
        rospy.logwarn("Learning predetermined task was interrupted (FAIL).")

    def start_task(self):
        # set dynamic reconfigure to default for initialization
        self.dyn_reconfig_client.update_configuration(self.default_config)

        # Creates a goal to send to the action server.
        goal = spc_uav_comm.msg.LearnParamGoal(start=True, **self.action_settings)

        # Sends the goal to the action server.
        self.action_client.send_goal(goal, done_cb=self._action_done_CB, feedback_cb=self._action_feedback_CB)
        self.action_running = True

        self.action_client.wait_for_result()

        result = self.action_client.get_result()

        self.reward = self.calculate_reward(result)
        print 'reward = ' + str(self.reward)

        # set dynamic reconfigure to default for ensure safety during transitions (Ramy)
        self.dyn_reconfig_client.update_configuration(self.default_config)

        return self.reward

    def cancel_task(self):
        self.default_dyn_reconfig()
        if self.action_running:
            self.action_client.cancel_goal()
            self.action_running = self.task_started = False

    def is_action_running(self):
        return self.action_running

    def calculate_reward(self, result):
        if not (result and result.finished):
            reward = self.failure_reward
            pos_error = np.zeros((1, 3))
            rot_error = np.zeros((1, 3))
        else:
            # Norm of Euclidean Distances along each axis:
            rot_error = np.empty((0, 3))
            for i in range(self.set_rot.shape[0]):
                R_set = tf.transformations.quaternion_matrix(self.set_rot[i])[0:3, 0:3]
                R_real = tf.transformations.quaternion_matrix(self.real_rot[i])[0:3, 0:3]
                R_err = 0.5*(np.matmul(R_set.transpose(), R_real) - np.matmul(R_real.transpose(), R_set))
                rot_error = np.concatenate((rot_error, np.array([R_err[2,1], R_err[0,2], R_err[1,0]]).reshape((1,3))))
            # pos_error = np.sqrt(np.sum((self.set_pos - self.real_pos) ** 2, 0)/self.real_pos.shape[0])
            # rot_error = np.sqrt(np.sum(rot_error**2, 0)/self.set_rot.shape[0])
            # pos_error = np.linalg.norm(self.set_pos - self.real_pos - [self.action_settings['depth'], 0, 0], axis=0)
            # rot_error = np.linalg.norm(rot_error, axis=0)
            pos_error = np.sqrt(np.sum((self.set_pos - self.real_pos - [self.action_settings['depth'], 0, 0])**2, 0)/self.set_pos.shape[0])
            rot_error = np.sqrt(np.sum(rot_error**2, 0)/self.set_rot.shape[0])
            vars = sum(self.current_config[var] for var in self.controlled_vars)

            print "pos_error = " + str(pos_error)
            print "rot_error = " + str(rot_error)
            print "sum of controlled variables values = " + str(vars)

            vars = vars/len(self.controlled_vars) if self.controlled_vars else 0
            reward = -100*sum(pos_error) - 150 * sum(rot_error) - 0.15*vars

        return reward, pos_error, rot_error

    def default_dyn_reconfig(self):
        self.current_config = self.default_config.copy()
        self.update_dyn_reconfig()

    def update_dyn_reconfig(self):
        self.dyn_reconfig_client.update_configuration(self.current_config)

    def add_dyn_ctr_var(self, var, min, max):
        if var in self.get_selectable_vars():
            self.controlled_vars[var] = (min, max)
            return True
        return False

    def add_dyn_fix_var(self, var):
        if var in self.get_selectable_vars():
            self.fixed_vars[var] = None
            return True
        return False

    def add_dyn_link_var(self, var1, var2):
        if var1 in self.get_selectable_vars() and var2 in self.get_linkable_vars():
            if var2 in self.linked_vars:
                self.linked_vars[var2].append(var1)
            else:
                self.linked_vars[var2] = [var1]
            self.links[var1] = var2
            if self.current_config[var1] != self.current_config[var2]:
                self.current_config[var1] = self.current_config[var2]
                self.update_dyn_reconfig()
            return True
        return False

    def set_dyn_reconfig_var(self, var, val=None):
        if var not in self.get_linkable_vars():
            return
        if val is None or (var in self.controlled_vars
                           and (val<self.controlled_vars[var][0] or val>self.controlled_vars[var][1])):
            val = self.default_config[var]
        if var in self.fixed_vars:
            self.fixed_vars[var] = val
        if self.current_config[var] != val:
            self.current_config[var] = val
            if var in self.linked_vars:
                for var2 in self.linked_vars[var]:
                    self.current_config[var2] = val
            self.update_dyn_reconfig()

    def send_controlled_vars(self, **vars):
        for var in vars:
            self.set_dyn_reconfig_var(var, vars[var])
        for var in self.fixed_vars:
            self.set_dyn_reconfig_var(var, self.fixed_vars[var])

    def get_selectable_vars(self):
        return list(set(self.available_vars) - set(self.controlled_vars.keys())
                           - set(self.links.keys()) - set(self.fixed_vars.keys()))

    def get_linkable_vars(self):
        return list(set(self.controlled_vars.keys()) | set(self.fixed_vars.keys()))

    def delete_dyn_reconfig_var(self, var):
        self.set_dyn_reconfig_var(var)
        if var in self.get_linkable_vars():
            if var in self.controlled_vars:
                del self.controlled_vars[var]
            elif var in self.fixed_vars:
                del self.fixed_vars[var]
            if var in self.linked_vars:
                for var2 in self.linked_vars[var]:
                    self.set_dyn_reconfig_var(var2)
                    del self.links[var2]
                del self.linked_vars[var]
        elif var in self.links:
            self.linked_vars[self.links[var]].remove(var)
            del self.links[var]

    def refresh_current_config(self):
        self.current_config = self.dyn_reconfig_client.get_configuration()


if __name__ == '__main__':
    try:
        learning_client = LearningClient()
        learning_gui = LearningGUI(learning_client)
    except rospy.ROSInterruptException:
        rospy.ERROR("program interrupted before completion")
