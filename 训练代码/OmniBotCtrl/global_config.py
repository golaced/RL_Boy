import os

ROOT_DIR = os.path.dirname(__file__)
ENVS_DIR = os.path.join(ROOT_DIR,'Env')
#Taitan Tinker TinkerXd Tinymal Liteloong
ROBOT_SEL = 'Tinker'
#Trot Stand
GAIT_SEL = 'Trot'
PLAY_DIR ='/home/rx/Downloads/OmniBotCtrl/logs/rough_go2_constraint/May14_09-16-46_test_barlowtwins/model_25000.pt'
SPD_X = 0.3
SPD_Y = 0.0
SPD_YAW = 0


#train param
MAX_ITER = 30000
SAVE_DIV = 5000


#./compile XX.urdf XX.xml
#rosrun robot_state_publisher robot_state_publisher my_robot.urdf