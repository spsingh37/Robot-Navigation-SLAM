import os
import re

# Set these variables to control which tasks to keep
# Default value is True, means that task will not be removed
tasks_to_keep = {
    # PART 2: SLAM
    'mapping': False,
    'action_model': False,
    'sensor_model': False,
    'particle_filter': False,
    # PART 3: Planning and Exploration
    'astar': False,
    'exploration': False
}

def remove_tasks(filename):
    with open(filename, 'r') as file:
        content = file.read()

    pattern = re.compile(r'// BEGIN (\w+).*?// END \1', re.DOTALL)
    content = pattern.sub(lambda match: match.group(0) if tasks_to_keep.get(match.group(1), False) else '', content)

    with open(filename, 'w') as file:
        file.write(content)

def process_directory(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.cpp'):
                remove_tasks(os.path.join(root, file))

if __name__ == '__main__':
    process_directory('./src')
