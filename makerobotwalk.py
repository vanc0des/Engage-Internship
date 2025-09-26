import sys, time
sys.path.append(r"C:\Users\vansh\OneDrive\Documents\Engage Scholarship\OpenCat-Quadruped-Robot-main\OpenCatPythonAPI")
import PetoiRobot as PR

# === Original walking gait (wkF) ===
wkF = [ 
39, 0, 0, 1,
  18,  54,  58,  52,   7,  13,  -2,   9,
  14,  56,  52,  54,  12,  14,  -3,  10,
  15,  57,  46,  55,  17,  15,  -4,  11,
  16,  58,  38,  57,  16,  17,  -2,  12,
  19,  59,  31,  58,  13,  19,   1,  14,
  22,  59,  24,  60,  12,  22,   6,  14,
  26,  60,  21,  61,  10,  24,  12,  16,
  28,  58,  23,  62,   9,  30,  16,  18,
  31,  61,  25,  63,   8,  30,  13,  20,
  34,  67,  28,  63,   7,  23,  12,  24,
  36,  69,  31,  63,   6,  15,  11,  27,
  38,  68,  34,  63,   7,   8,  10,  31,
  41,  65,  36,  62,   7,   3,   9,  35,
  43,  60,  39,  65,   7,  -1,   9,  37,
  46,  55,  41,  70,   7,  -3,   9,  27,
  47,  50,  43,  71,   7,  -5,   9,  23,
  49,  43,  47,  71,   8,  -5,   7,  14,
  51,  35,  48,  69,   8,  -3,   8,   8,
  53,  28,  51,  66,   9,   1,   8,   2,
  54,  18,  52,  63,  12,   7,   9,  -2,
  55,  15,  54,  56,  14,  10,  10,  -3,
  56,  13,  55,  51,  15,  16,  11,  -4,
  57,  16,  57,  43,  17,  16,  12,  -3,
  58,  18,  58,  36,  19,  14,  14,  -2,
  59,  21,  60,  28,  21,  12,  14,   2,
  60,  25,  61,  23,  24,  10,  16,   9,
  60,  28,  62,  22,  27,   9,  18,  15,
  60,  31,  63,  23,  30,   8,  20,  16,
  66,  32,  63,  26,  24,   7,  24,  13,
  68,  35,  63,  29,  19,   6,  27,  11,
  68,  38,  63,  32,  10,   6,  31,  10,
  66,  40,  62,  35,   4,   7,  35,  10,
  62,  42,  65,  37,   0,   7,  37,   9,
  57,  45,  70,  39,  -2,   7,  27,   9,
  52,  47,  71,  41,  -5,   7,  23,   9,
  45,  48,  71,  44,  -5,   8,  14,   9,
  38,  51,  69,  47,  -3,   8,   8,   7,
  30,  52,  66,  49,  -1,   9,   2,   8,
  22,  53,  63,  51,   4,  12,  -2,   8,
]

# === Load gallop gait from file ===
def load_gallop_gait():
    """Load gallop gait from the generated text file"""
    try:
        with open("gallop_gait.txt", "r") as f:
            lines = f.readlines()
        
        # Find the start of the array data
        glF = []
        in_array = False
        
        for line in lines:
            line = line.strip()
            if line.startswith("glF = ["):
                in_array = True
                continue
            elif line == "]":
                break
            elif in_array and line and not line.startswith("#"):
                # Extract numbers from the line
                if "," in line:
                    # Remove comments and split by comma
                    data_part = line.split("#")[0].strip()
                    if data_part.endswith(","):
                        data_part = data_part[:-1]  # Remove trailing comma
                    
                    # Split by comma and convert to integers
                    numbers = [int(x.strip()) for x in data_part.split(",") if x.strip()]
                    glF.extend(numbers)
        
        if glF:
            print(f"✅ Gallop gait loaded: {len(glF)} total values, {glF[0]} frames")
            return glF
        else:
            print("❌ No gallop gait data found in file")
            return None
            
    except FileNotFoundError:
        print("❌ gallop_gait.txt not found. Please run the transformation script first.")
        return None
    except Exception as e:
        print(f"❌ Error loading gallop gait: {e}")
        return None

# Load the gallop gait at startup
glF = load_gallop_gait()

def connect_to_robot():
    """Connect to the robot"""
    print("🔌 Connecting to robot...")
    PR.autoConnect()
    time.sleep(2)  # Give time for connection
    
    if PR.goodPorts:
        print(f"✅ Connected to robot on {list(PR.goodPorts.keys())[0]}")
        return True
    else:
        print("❌ Failed to connect to robot")
        return False

def play_gait(gait_array, gait_name, duration=5):
    """Play the specified gait sequence"""
    if not gait_array:
        print(f"❌ {gait_name} gait not available")
        return False
        
    print(f"🚶 Playing {gait_name} sequence...")
    
    try:
        # First, put the robot in a balanced state
        print("⚖️  Putting robot in balanced state...")
        PR.send(PR.goodPorts, ['kbalance', 1])
        time.sleep(1)
        
        # Send the gait skill with duration parameter
        PR.send(PR.goodPorts, ['K', gait_array, duration])
        
        # Wait for the skill to complete
        time.sleep(duration + 1)  # Slightly longer than the skill duration
        
        # Return to balanced state
        print("⚖️  Returning to balanced state...")
        PR.send(PR.goodPorts, ['kbalance', 1])
        time.sleep(1)
        
        print(f"✅ {gait_name} sequence completed!")
        return True
        
    except Exception as e:
        print(f"❌ Error during {gait_name} execution: {e}")
        return False

def get_user_choice():
    """Get user's choice for gait type"""
    print("\n" + "="*50)
    print("🤖 PETOI ROBOT GAIT CONTROLLER")
    print("="*50)
    print("Choose gait type:")
    print("1. Walk (wkF) - Original walking gait")
    
    if glF:
        print("2. Gallop (glF) - Transformed gallop gait")
    else:
        print("2. Gallop (glF) - ❌ Not available (run transformation script first)")
    
    print("3. Exit")
    print("-" * 50)
    
    while True:
        try:
            choice = input("Enter your choice (1-3): ").strip()
            
            if choice == "1":
                return "walk"
            elif choice == "2":
                if glF:
                    return "gallop"
                else:
                    print("❌ Gallop gait not available. Please run the transformation script first.")
                    continue
            elif choice == "3":
                return "exit"
            else:
                print("❌ Invalid choice. Please enter 1, 2, or 3.")
                continue
                
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            return "exit"

def main():
    """Main execution function"""
    print("🚀 Starting Petoi Robot Gait Controller...")
    
    # Check if robot connection is available
    if not connect_to_robot():
        print("\n❌ Could not connect to robot. Please check:")
        print("1. Is the robot turned on and connected via USB?")
        print("2. Are the batteries charged?")
        print("3. Try restarting the robot and running the script again.")
        return
    
    try:
        while True:
            choice = get_user_choice()
            
            if choice == "exit":
                break
            elif choice == "walk":
                print("\n🚶 Selected: Walk (wkF)")
                play_gait(wkF, "Walking", duration=5)
            elif choice == "gallop":
                print("\n🏃 Selected: Gallop (glF)")
                play_gait(glF, "Galloping", duration=5)
            
            # Ask if user wants to continue
            print("\n" + "-" * 30)
            continue_choice = input("Would you like to try another gait? (y/n): ").strip().lower()
            if continue_choice not in ['y', 'yes']:
                break
    
    except KeyboardInterrupt:
        print("\n👋 Program interrupted by user")
    
    finally:
        # Ensure we stop any motion and close the connection
        try:
            print("🛑 Stopping robot motion...")
            PR.send(PR.goodPorts, ['i', 0])
            time.sleep(1)
            PR.closePort()
            print("✅ Connection closed safely")
        except:
            pass
    
    print("👋 Goodbye!")

if __name__ == "__main__":
    main()
