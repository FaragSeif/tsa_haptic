# tsa_haptic
 A collection of modules and scripts for a TSA-based Haptic device


 ```bash 
├── docs # documentation
│
├── topside  
│   ├── user_interface #user_interface
│   ├── logger # 
│   └── printer
│
├── communication
│   ├── lcm 
│   └── udp
│
├── device
│   ├── interface 
│   └── misc
│  

│  
│  
├── model
│   ├── utils
│   ├── simulator  
│   ├── kinematics 
│   └── dynamics 
│
├── hardware # набор модулей отвечающих за интерфейс к железу
│   ├── sensor_board # интерфейс к джойстику
│   ├── bluerov # интерфейс к blue rov 
│   ├── pwm_driver # интерфейс
│   ├── imu # интерфейс к imu
│   ├── ping360 # интерфейс к сонару ping 360
│   └── underwater_gps # интерфейс к подводному gps waterlinked G2
│
├── examples 
│
└── tests
```