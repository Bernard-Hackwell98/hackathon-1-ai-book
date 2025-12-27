# Quickstart: ROS 2 for Humanoid Robotics Module

## Prerequisites

- ROS 2 Humble Hawksbill installed (with Python 3.8+)
- Node.js 18+ and npm/yarn
- Git
- Basic Python knowledge

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Docusaurus Dependencies
```bash
npm install
```

### 3. Install ROS 2 Dependencies
```bash
# Install ROS 2 Humble Hawksbill following official instructions
# https://docs.ros.org/en/humble/Installation.html

# Install rclpy (Python ROS client library)
sudo apt install python3-ros-environment python3-ros-workspace python3-roslib python3-rospkg python3-rosdep
```

### 4. Install Gazebo (Simulation Environment)
```bash
# Install Ignition Gazebo Fortress
sudo apt install ignition-fortress
```

## Running the Documentation

### 1. Start Docusaurus Development Server
```bash
npm start
```

This will start a local development server at `http://localhost:3000`.

### 2. Building for Production
```bash
npm run build
```

## Running ROS 2 Examples

### 1. Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### 2. Navigate to Example Directory
```bash
cd docs/modules/ros2-humanoid/chapter-2-communication/examples
```

### 3. Run Python Examples
```bash
python3 simple_publisher.py
python3 simple_subscriber.py
```

## Creating New Content

### 1. Adding a New Chapter
1. Create a new directory in `docs/modules/ros2-humanoid/`
2. Add an `index.md` file with the chapter content
3. Update the sidebar configuration in `sidebars.js`

### 2. Adding Code Examples
1. Create an `examples/` directory within your chapter directory
2. Add Python files with `.py` extension
3. Include setup instructions and expected output in comments

## Testing Your Changes

### 1. Documentation Validation
```bash
npm run build  # Should complete without errors
```

### 2. Link Validation
```bash
npm run serve  # Serve the built site and verify all links work
```

### 3. Example Code Validation
1. Test each code example in a clean environment
2. Verify that setup instructions are complete and accurate
3. Confirm expected outputs match actual outputs

## Deployment

The documentation is deployed to GitHub Pages. To deploy:

1. Build the site: `npm run build`
2. Push changes to the `main` branch
3. GitHub Actions will automatically deploy to the configured GitHub Pages URL

## Troubleshooting

### Common Issues

1. **Python import errors**: Ensure ROS 2 environment is sourced (`source /opt/ros/humble/setup.bash`)
2. **Docusaurus build errors**: Check that all Markdown files have proper frontmatter
3. **Gazebo simulation issues**: Verify Gazebo installation and ROS 2 integration

### Getting Help

- Check the ROS 2 documentation: https://docs.ros.org/
- Docusaurus documentation: https://docusaurus.io/
- Create an issue in the repository for specific problems with this module