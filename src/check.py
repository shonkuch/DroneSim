from pymavlink import mavutil

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=30)
print("OK heartbeat", m.target_system, m.target_component)
