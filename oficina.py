from pymavlink import mavutil
import threading
import time
import math
import numpy as np

# Variaveis Globais
encerra_script = 0

# Configurar a conexão com o drone usando PyMAVLink
def vehicle_connection(option = "sitl", mavlink_id = 1):
    match option:
        case "mavproxy":
            connect_string = 'udp:0.0.0.0:14551'       # for SITL 
        case "sitl":
            connect_string = 'tcp:127.0.0.1:5763'      # for SITL 
        case "serial":
            connect_string = '/dev/ttyAMA0'            # for RPi
        case _:
            print("\nConexão inválida!")

    # Start a connection
    print("Connecting on: ", connect_string)
    vehicle = mavutil.mavlink_connection(connect_string, baud=57600, 
                                        source_system=mavlink_id, source_component=2)
    
    # Wait for the first heartbeat
    vehicle.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, 
                                                              vehicle.target_component))    
    return vehicle

# Definir Home e EKF origin (Posiciona o drone sem GPS)
def set_home_and_ekf_origin(vehicle, latitude=-27.5937609, longitude=-48.5417189, altitude=0, timestamp=1715363270):
    # Send MAVLink Comand
    vehicle.mav.command_long_send(
        vehicle.target_system,                # Target system
        vehicle.target_component,             # Target component
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
        0,                                    # confirmation
        0,                                    # param1 (1=use current location, 0=use specified location)
        0,                                    # not used
        0,                                    # not used
        0,                                    # not used
        latitude,                             # param5 (Latitude in degrees)
        longitude,                            # param6 (Longitude in degrees)
        altitude                              # param7 (Altitude in meters)
    )

    # Send MAVLink Message
    vehicle.mav.set_gps_global_origin_send(
        vehicle.target_system,                # Target system
        int(latitude*1e7),                    # Latitude in degrees
        int(longitude*1e7),                   # Longitude in degrees
        0,                                    # Altitude in meters
        int(timestamp)                        # timestamp 
    )
    
# Requisicao para receber mensagens MAVLink
def request_data_stream(vehicle):
    # Class for msg request configuration
    class rqt_msg_config:
        def __init__(self,msg_id,msg_interval):
            self.msg_id = msg_id                # mavlink message ID
            self.msg_interval = msg_interval    # message interval in us
    
    # List of MAVLink messages request
    rqt_msg = []
    rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,  msg_interval = 1e6/4))
    rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, msg_interval = 1e6/4))
    rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,msg_interval = 1e6/2))
    rqt_msg.append(rqt_msg_config(msg_id = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,msg_interval = 1e6/2))

    # Request each msg in the list
    for rqt in rqt_msg:
        # MAVLink command for SET_MESSAGE_INTERVAL
        vehicle.mav.command_long_send(
            vehicle.target_system,      # Target system ID
            vehicle.target_component,   # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            0,                          # Confirmation
            rqt.msg_id,                 # param1: Message ID to be streamed
            rqt.msg_interval,           # param2: Interval in microseconds
            0,                          # param3 (unused)
            0,                          # param4 (unused)
            0,                          # param5 (unused)
            0,                          # param5 (unused)
            0                           # param6 (unused)
        )

    print('Aguardando mensagens MAVLink')
    time.sleep(5)

# Recebe dados do drone
def listener(vehicle):
    # Esta funcao foi criada para rodar em uma thread separada
    while not encerra_script:
        msg = vehicle.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            vehicle.roll        = msg.roll          # [rad]
            vehicle.pitch       = msg.pitch         # [rad]
            vehicle.yaw         = msg.yaw           # [rad]
            vehicle.rollspeed   = msg.rollspeed     # [rad/s]
            vehicle.pitchspeed  = msg.pitchspeed    # [rad/s]
            vehicle.yawspeed    = msg.yawspeed      # [rad/s]

        msg = vehicle.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            vehicle.airspeed    = msg.airspeed      # [m/s]
            vehicle.groundspeed = msg.groundspeed   # [m/s]
            vehicle.heading     = msg.heading       # [deg]
            vehicle.throttle    = msg.throttle      # [%]
            vehicle.alt         = msg.alt           # [m]   - MSL
            vehicle.climb       = msg.climb         # [m/s] - climb rate    

        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            vehicle.lat         = msg.lat                # [1e7]
            vehicle.lon         = msg.lon                # [1e7]
            vehicle.relative_alt= msg.relative_alt*0.001 # [m]
            vehicle.vx          = msg.vx*0.01            # [m/s]
            vehicle.vy          = msg.vy*0.01            # [m/s]
            vehicle.vz          = msg.vz*0.01            # [m/s]
        
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            vehicle.x          = msg.x  # [m]
            vehicle.y          = msg.y  # [m]
            vehicle.z          = msg.z  # [m]

        time.sleep(0.1)

# Funcao para Armar e Decolar
def arm_and_takeoff(vehicle, target_altitude):
    # Mode = GUIDED
    vehicle.set_mode("GUIDED")
    time.sleep(0.2)
    
    # Arm drone
    vehicle.arducopter_arm()
    print("Armando..")

    # Wait until vehicle armed
    vehicle.motors_armed_wait()
    print("Drone armado")

    # Send TakeOff command message
    vehicle.mav.command_long_send(
        vehicle.target_system,      # ID do sistema de destino
        vehicle.target_component,   # ID do componente de destino
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Comando de decolagem
        0,                          # Confirmation
        0,                          # Param 1 (Desconsiderado)
        0,                          # Param 2 (Desconsiderado)
        0,                          # Param 3 (Desconsiderado)
        0,                          # Param 4 (Desconsiderado)
        0,                          # Param 5 (Latitude, Desconsiderado)
        0,                          # Param 6 (Longitude, Desconsiderado)
        target_altitude             # Param 7 (Altura de decolagem em metros)
    )

    # Wait until reach target altitude
    while vehicle.relative_alt < target_altitude*0.95:
        print("Altitude: ", vehicle.relative_alt)
        time.sleep(0.5)

    print("Altitude atingida")

# Envia posicao desejada em coordendas (lat, lon) 
def send_position_target_global_int(vehicle, 
                                   lat_int,   lon_int,  alt=None,
                                   vx=0.0,    vy=0.0,   vz=0.0,
                                   afx=0.0,   afy=0.0,  afz=0.0,
                                   yaw_angle = None,yaw_rate = 0.0):
    """
    yaw_angle in [degrees]
    yaw_rate in [degrees/s]
    velocities and aceletations ignored
    """
    if alt is None:
        # Holds original alt
        alt = vehicle.relative_alt
    
    if yaw_angle is None:
        # Holds original heading (yaw is more precise than heading)
        yaw_angle = math.degrees(vehicle.yaw)

    # Send MAVLink Message
    vehicle.mav.set_position_target_global_int_send(
        0,                          # time_boot_ms
        vehicle.target_system,      # Target system
        vehicle.target_component,   # Target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        #0b101111111000,         # Bitmap to indicate which dimensions should be ignored by the vehicle.
        0b111111111000,         # Ignora YAW => Drone anda sempre de frente (vira na direcao do movimento)
        lat_int,                # Longitude times 1e7   [integer]
        lon_int,                # Latitude  times 1e7   [integer]
        alt,                    # Altitude Relative       [m]
        vx,                     # X velocity in NED frame [m/s]
        vy,                     # Y velocity in NED frame [m/s]
        vz,                     # Z velocity in NED frame [m/s]
        afx,                    # X acceleration or force [m/s2]
        afy,                    # Y acceleration or force [m/s2]
        afz,                    # Z acceleration or force [m/s2]
        math.radians(yaw_angle),# Yaw setpoint [rad]
        math.radians(yaw_rate)  # Body yaw rate [rad/s]
    )

# Envia posicao desejada no frame NED (x,y,z)
def send_position_target_local_ned(vehicle, 
                                   x,         y,        z=None,
                                   vx=0.0,    vy=0.0,   vz=0.0,
                                   afx=0.0,   afy=0.0,  afz=0.0,
                                   yaw_angle = None,yaw_rate = 0.0):
    """
    z is negative
    yaw_angle in [degrees]
    yaw_rate in [degrees/s]
    velocities and aceletations ignored
    """
    if z is None:
        # Holds original alt
        z = -vehicle.relative_alt # NED => z is down (negative values)
    
    if yaw_angle is None:
        # Holds original heading (yaw is more precise than heading)
        yaw_angle = math.degrees(vehicle.yaw)
    
    print(x,y,z)

    # Send MAVLink Message
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms
        vehicle.target_system,      # Target system
        vehicle.target_component,   # Target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # coordinate_frame (with origin fixed relative to earth.)
        #0b101111111000,         # Bitmap to indicate which dimensions should be ignored by the vehicle.
        0b111111111000,         # Ignora YAW => Drone anda sempre de frente (vira na direcao do movimento)
        x,                      # X Position in NED frame [m]
        y,                      # Y Position in NED frame [m]
        z,                      # Z Position in NED frame [m]
        vx,                     # X velocity in NED frame [m/s]
        vy,                     # Y velocity in NED frame [m/s]
        vz,                     # Z velocity in NED frame [m/s]
        afx,                    # X acceleration or force [m/s2]
        afy,                    # Y acceleration or force [m/s2]
        afz,                    # Z acceleration or force [m/s2]
        math.radians(yaw_angle),# Yaw setpoint [rad]
        math.radians(yaw_rate)  # Body yaw rate [rad/s]
    )

# Conversao de frames
def drone_frame_to_local_ned(drone_frame_coord, drone_yaw):
    """
    drone_frame_coord = (x,y,z)
    drone_yaw in radians
    """
    # Class for 3d position
    class pos3:
        def __init__(self,x,y,z):
            self.x = x
            self.y = y
            self.z = z

    rotation_matrix = np.array([
        [np.cos(drone_yaw), -np.sin(drone_yaw), 0],
        [np.sin(drone_yaw), np.cos(drone_yaw), 0],
        [0, 0, 1]
    ])
    local_ned_coordinates = np.dot(rotation_matrix, drone_frame_coord)
    return pos3(local_ned_coordinates[0],local_ned_coordinates[1],local_ned_coordinates[2])

# Aguarda chegar na posicao solicitada (coordenada)
def position_target_local_ned_wait(vehicle, target_coord, tolerance, timeout):
    i=0
    while i<timeout:
        if ((abs(target_coord.x-vehicle.x)<=tolerance) and 
        (abs(target_coord.y-vehicle.y)<=tolerance)):
            print("Target position reached!")
            return
        i+=1
        time.sleep(1)
    print('Position Target [TIMEOUT=',i,'s]')

# Aguarda chegar na posicao solicitada (x,y,z)
def position_target_global_int_wait(vehicle, lat, lon, tolerance, timeout):
    i=0
    while i<timeout:
        if ((abs(lat-vehicle.lat)<=tolerance) and 
        (abs(lon-vehicle.lon)<=tolerance)):
            print("Target position reached!")
            return
        i+=1
        time.sleep(1)
    print('Position Target [TIMEOUT=',i,'s]')

#######################################################################################################################
##                                              CODE STARTS HERE
#######################################################################################################################

# Conexao com o drone
drone = vehicle_connection("sitl")

# Solicita as mensagens MAVLink 
request_data_stream(drone)

# Cria a thread Listener de mensagens MAVLink
thread_listener = threading.Thread(target=listener, args=(drone,))
thread_listener.daemon = True  # Define a thread como um daemon para que ela seja encerrada quando o programa principal terminar
thread_listener.start()

# Decola para 5m de altura
arm_and_takeoff(vehicle=drone, target_altitude= 5.0)

# Espera pairado por 5s
print('Pairando')
time.sleep(5)

# # Move pela Posicao Relativa
# target_position_drone = (5,0,0)                                                # (x,y,z) in drone frame [m]
# target_position_ned = drone_frame_to_local_ned(target_position_drone, drone.yaw)# (N,E,D) in local ned frame (ajusta heading)
# target_position_ned.x += drone.x    # Soma posicao atual do drone com o incremento (Posicao relativa)
# target_position_ned.y += drone.y    # Soma posicao atual do drone com o incremento (Posicao relativa)
# target_position_ned.z += drone.z    # Soma posicao atual do drone com o incremento (Posicao relativa)
# print('Iniciando movimento')
# send_position_target_local_ned(drone, target_position_ned.x, target_position_ned.y)
# position_target_local_ned_wait(drone, target_position_ned,tolerance=0.5,timeout=120)
# print('Posicao atingida')

# Move pela Coordenada
print('Iniciando movimento')
target_coord = (-275935589,-485417027)  # (Latitude, Longitude) - Base 2
# target_coord = (-275937609,-485417189)  # (Latitude, Longitude) - Base 1
send_position_target_global_int(drone, target_coord[0], target_coord[1])  
position_target_global_int_wait(drone, target_coord[0], target_coord[1], tolerance=50, timeout=120)
print('Posicao atingida')

# Espera pairado por 5s
print('Pairando')
time.sleep(5)

# Pousar
print('Pousando')
drone.set_mode("LAND")
drone.motors_disarmed_wait()
print('Desarmado')

# Aguarda o termino da thread para finalizar
encerra_script = 1
thread_listener.join()
print("FIM")