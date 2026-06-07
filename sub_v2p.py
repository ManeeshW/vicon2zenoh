import zenoh
import json
import time
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Create a Zenoh configuration (default should work for local or peer-to-peer)
conf = zenoh.Config()

# Open a Zenoh session
session = zenoh.open(conf)

# Lists to store data for plotting
timestamps = []
positions = []
first_timestamp = None

# Define the callback function to handle incoming samples
def listener(sample):
    global first_timestamp
    try:
        # Decode the payload (assuming it's JSON encoded as a string)
        payload_str = sample.payload.to_string()
        data = json.loads(payload_str)
        
        # Extract relevant fields
        image_taken_time = data.get('image_taken_time')
        pose = data.get('pose')  # 3x4 list of lists
        
        # Extract position from pose (last column)
        x = pose[0][3]
        y = pose[1][3]
        z = pose[2][3]
        
        # Set first timestamp if not set
        if first_timestamp is None:
            first_timestamp = image_taken_time
        
        # Store data
        timestamps.append(image_taken_time)
        positions.append([x, y, z])
        
        # Print for logging
        print(f"Received position: x={x}, y={y}, z={z} at timestamp={image_taken_time}")
        
    except Exception as e:
        print(f"Error processing sample: {e}")

# Declare the subscriber on the key expression
key_expr = 'fdcl/pose_sync'  # Default key from config
sub = session.declare_subscriber(key_expr, listener)

# Start time for 100s duration
start_time = time.time()

# Keep the subscriber running for 100 seconds
print(f"Subscribing to '{key_expr}' for 100 seconds...")
try:
    while time.time() - start_time < 100:
        time.sleep(0.1)  # Check every 0.1 seconds
except KeyboardInterrupt:
    print("Interrupted by user.")

# Clean up
sub.undeclare()
session.close()

# Generate Plotly plot if data was received
if timestamps and positions:
    # Convert timestamps to relative seconds
    relative_times = [(ts - first_timestamp) / 1e9 for ts in timestamps]
    
    # Create Plotly figure with subplots
    fig = make_subplots(
        rows=3, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.1,
        subplot_titles=('X Position', 'Y Position', 'Z Position')
    )
    
    # Add traces
    fig.add_trace(go.Scatter(x=relative_times, y=[p[0] for p in positions], mode='lines', name='X Position'), row=1, col=1)
    fig.add_trace(go.Scatter(x=relative_times, y=[p[1] for p in positions], mode='lines', name='Y Position'), row=2, col=1)
    fig.add_trace(go.Scatter(x=relative_times, y=[p[2] for p in positions], mode='lines', name='Z Position'), row=3, col=1)
    
    # Update layout
    fig.update_layout(
        title='Position over Time',
        height=800,  # Adjust height for subplots
        showlegend=False  # Hide legend since titles are on subplots
    )
    fig.update_xaxes(title_text='Time (seconds)', row=3, col=1)
    fig.update_yaxes(title_text='Position', row=1, col=1)
    fig.update_yaxes(title_text='Position', row=2, col=1)
    fig.update_yaxes(title_text='Position', row=3, col=1)
    
    # Save to HTML
    fig.write_html("position_plot.html")
    print("Plot saved to 'position_plot.html'")
else:
    print("No data received during the 100 seconds.")