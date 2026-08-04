from lerobot.teleoperators.so_leader import SO101LeaderConfig
from lerobot.teleoperators.so_leader.so_leader import SO101Leader 

PORT = "/dev/guardian_right_leader"  # Replace with the actual port for the right leader
print(f"Testing right leader on port: {PORT}")

# Create ONLY the right leader config
config = SO101LeaderConfig(port=PORT, id="test_right_leader")
teleop = SO101Leader(config)

try:
    teleop.connect() 
    print("✅ SUCCESS! The right leader motors responded.")
    teleop.disconnect()
except Exception as e:
    print(f"❌ FAILED to connect: {e}")