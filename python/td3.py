class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()

        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a
# TD3 network
class TD3(object):
    def __init__(self, state_dim, action_dim):
        # Initialize the Actor network
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
        self.actor = Actor(state_dim, action_dim)
        self.actor.load_state_dict(torch.load("/home/polya/DRL-robot-navigation-main/pytorch_models180/TD3_final_actor.pth"))
        self.actor = self.actor.to(self.device)
        self.max_action = 1
    def get_action(self, state):
        # Function to get the action from the actor
        state = torch.Tensor(state.reshape(1, -1)).to(self.device)
        action = self.actor(state).cpu().data.numpy().flatten()
        action = action.clip(-self.max_action,self.max_action)
        linear_v = (action[0]+1)/2
        angle_v = action[1]
        return linear_v,angle_v
    def load(self, filename, directory):
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename))
        )


        #-------------------Pub and Sub--------------
        self.GoalPub = rospy.Publisher("/GoalPoint",Marker,queue_size=10)
        self.LaserSub = rospy.Subscriber("/scan",LaserScan,self.LaserScanCallBack)
        self.OdomSub = rospy.Subscriber("/odom",Odometry,self.OdomCallBack)
        self.goalxSub = rospy.Subscriber("/goal_x",Float32,self.goalxCallBack)
        self.goalySub = rospy.Subscriber("/goal_y",Float32,self.goalyCallBack)



    def GetStates(self): #其实这里和之前的velody_env.py中获取信息的方式大同小异
        target =False
        v_state = []
        v_state[:] = self.velodyne_data[:]
        #雷达状态输入
        laser_state = [v_state]
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta
        self.dis = distance
        if distance < GOAL_REACHED_DIST:
            target = True
      	#机器人状态输入
        robot_state = [distance, theta, self.last_odom.twist.twist.linear.x,self.last_odom.twist.twist.angular.z]
        print(self.dis)
        print(distance)
        print(robot_state)
        state = np.append(laser_state,robot_state)
        # 最后我们返回小车的状态输入，包括雷达输入和机器人状态输入以及是否到达了目标位置
        return state,target
    
rospy.init_node("my_car") #初始化节点
car = mycar()	#实例化实车信息获取类
# --------偷懒的目标点发布---------------
xpub = rospy.Publisher("/goal_x",Float32,queue_size=10)
ypub = rospy.Publisher("/goal_y",Float32,queue_size=10)
xpub.publish(2.5)
ypub.publish(0.5)
network = TD3(24,2) #实例化TD3网络
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10) #/cmd_vel发布，用于控制小车的运动
rate = rospy.Rate(10) #等待频率，相当于仿真环境中的DELTA_TIME

while not rospy.is_shutdown():
    state,target = car.GetStates() #获取状态信息
    if not target:
        linear_x,angular_z = network.get_action(np.array(state)) #将状态输入到网络之中，输出线速度和角速度
        cmd_vel = Twist() #将速度放到Twist中发布
        cmd_vel.linear.x = linear_x/5 #这里发现最快1m/s的话还是太快了，所以又手动降为最大0.2
        cmd_vel.angular.z = angular_z/5
        pub.publish(cmd_vel) #发布消息
        
    if state[20]<COLLISION_DIST: #简易判断碰撞
        pub.publish(Twist())
        break
    if target: #简易判断是否到达目标点
        pub.publish(Twist())
        break
    rate.sleep() #停止一段时间，让这里设置为0.1s，那么控制频率就是10hz
