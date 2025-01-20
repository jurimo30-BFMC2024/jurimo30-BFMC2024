import os

def main():
    package_name = input("Enter the package name: ")

    directory_path = f"{package_name}"
    os.makedirs(directory_path, exist_ok=True)

    file_path = os.path.join(directory_path, f"{package_name}.py")
    with open(file_path, 'w') as file:
        file.write(f'from src.utils.messages.allMessages import (mainCamera)\n')
        file.write(f'from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber\n')
        file.write(f'from src.utils.messages.messageHandlerSender import messageHandlerSender\n\n')
        file.write(f'class {package_name}():\n')
        file.write(f'    """This thread handles {package_name}.\n')
        file.write(f'    Args:\n')
        file.write(f'        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.\n')
        file.write(f'        logging (logging object): Made for debugging.\n')
        file.write(f'        debugging (bool, optional): A flag for debugging. Defaults to False.\n')
        file.write(f'    """\n\n')
        file.write(f'    def __init__(self, queueList, logging, debugging=False):\n')
        file.write(f'        self.queuesList = queueList\n')
        file.write(f'        self.logging = logging\n')
        file.write(f'        self.debugging = debugging\n')
        file.write(f'        self.subscribe()\n\n')
        file.write(f'    def run(self):\n')
        file.write(f'        pass\n\n')
        file.write(f'    def subscribe(self):\n')
        file.write(f'        """Subscribes to the messages you are interested in"""\n')
        file.write(f'        pass\n')

    autoFSM_path = "autoFSM.py"
    if not os.path.exists(autoFSM_path):
        print("The main.py file does not exist.")
        return
    
    with open(autoFSM_path, 'r') as file:
        lines = file.readlines()

    import_line = f"from src.core.Auto.{package_name}.{package_name} import {package_name}\n"
    lines.insert(0, import_line)

    # Write back to main.py
    with open(autoFSM_path, 'w') as file:
        file.writelines(lines)

    print("File created and main.py updated.")

if __name__ == "__main__":
    main()