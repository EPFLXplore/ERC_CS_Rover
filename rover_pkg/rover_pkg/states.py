from enum import Enum

class SubSystems(Enum):
    NAVIGATION = 0,
    HANDLING_DEVICE = 1,
    DRILL = 2,
    AVIONICS = 3,
    
'''
Led System

- Default: Mode when the subsystem is either not in use, or in just powerered on
- Manual: Mode when manual is on
- Auto: Mode when auto is on
- Fault: Mode when the subsystem is in fault
- Reset Motors: Mode when the motors need to be reset by someone near the rover
- Emergency Shutdown: Mode when the rover has to be shut down 

'''
class LedMode(Enum):
    OFF = 0,
    MANUAL = 1,
    AUTO = 2,
    FAULT = 3
