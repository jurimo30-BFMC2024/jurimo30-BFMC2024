#!/usr/bin/env python3
"""
Demonstration script for the 'kako ide' system status feature.
This shows the kind of data that would be collected and displayed.
"""

import json
import time
import sys
import os

# Add the src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

def simulate_system_status():
    """Simulate the system status data that would be collected"""
    current_time = time.strftime("%Y-%m-%d %H:%M:%S")
    uptime = 3672  # Example: about 1 hour uptime
    
    # Simulate some system metrics
    cpu_usage = 45.2  # 45.2% CPU usage
    memory_usage = 62.8  # 62.8% memory usage  
    temperature = 55  # 55°C temperature
    
    # Assess system health
    cpu_ok = cpu_usage < 90
    memory_ok = memory_usage < 85
    temp_ok = temperature < 70
    
    overall_status = "dobro" if (cpu_ok and memory_ok and temp_ok) else "paznja"
    
    # Simulate process activity
    active_processes = 8
    total_processes = 10
    
    status_data = {
        "timestamp": current_time,
        "uptime_seconds": uptime,
        "overall_status": overall_status,
        "system_health": {
            "cpu_ok": cpu_ok,
            "memory_ok": memory_ok,
            "temperature_ok": temp_ok
        },
        "active_processes": active_processes,
        "total_processes": total_processes,
        "message_activity": {
            "serialCamera": "active",
            "LaneDetect": "active",
            "ObjectDetection": "active",
            "CoreControl": "active",
            "BatteryLvl": "active",
            "Location": "inactive",
            "Cars": "inactive",
            "Semaphores": "active"
        },
        "performance": {
            "cpu_usage": cpu_usage,
            "memory_usage": memory_usage,
            "temperature": temperature
        }
    }
    
    return status_data

def format_uptime(seconds):
    """Format uptime in a human-readable way"""
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    secs = seconds % 60
    return f"{hours}h {minutes}m {secs}s"

def main():
    print("=== KAKO IDE - SYSTEM STATUS DEMO ===")
    print("Demonstrating the autonomous vehicle system status monitoring\n")
    
    # Import our new message type
    try:
        from src.utils.messages.allMessages import SystemStatus
        print(f"✓ SystemStatus message type imported successfully")
        print(f"  Queue: {SystemStatus.Queue.value}")
        print(f"  Owner: {SystemStatus.Owner.value}")
        print(f"  Message ID: {SystemStatus.msgID.value}")
        print(f"  Message Type: {SystemStatus.msgType.value}")
        print()
    except ImportError as e:
        print(f"✗ Could not import SystemStatus: {e}")
        return
    
    # Generate sample status data
    status = simulate_system_status()
    
    print("Sample system status data that would be sent to the dashboard:")
    print("=" * 60)
    print(json.dumps(status, indent=2, ensure_ascii=False))
    print("=" * 60)
    
    # Show how it would appear in Serbian/Croatian
    print("\nKako ide - System Status Summary:")
    print("-" * 40)
    print(f"Vreme: {status['timestamp']}")
    print(f"Uptime: {format_uptime(status['uptime_seconds'])}")
    print(f"Status: {status['overall_status']}")
    print(f"Procesi: {status['active_processes']}/{status['total_processes']}")
    print(f"CPU: {status['performance']['cpu_usage']:.1f}%")
    print(f"Memorija: {status['performance']['memory_usage']:.1f}%")
    print(f"Temperatura: {status['performance']['temperature']}°C")
    
    health_msg = "✓ Sistem radi dobro" if status['overall_status'] == 'dobro' else "⚠ Pažnja potrebna"
    print(f"\n{health_msg}")
    
    print(f"\nActive processes:")
    for process, activity in status['message_activity'].items():
        indicator = "●" if activity == "active" else "○"
        print(f"  {indicator} {process}: {activity}")

if __name__ == "__main__":
    main()