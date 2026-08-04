# Import paths matched to your config.py
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower 

PORT = "/dev/guardian_left_follower"
print(f"Testing right arm on port: {PORT}")

# Create ONLY the right arm config (no cameras)
config = SO101FollowerConfig(port=PORT, id="test_right")
robot = SO101Follower(config)

try:
    robot.connect() 
    print("✅ SUCCESS! The right arm motors responded and should now be stiff.")
    
    import time
    time.sleep(5) 
    
    robot.disconnect()
    print("Disconnected. Motors should be loose again.")
except Exception as e:
    print(f"❌ FAILED to connect: {e}")