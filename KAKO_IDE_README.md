# Kako Ide - System Status Monitoring

This implementation adds a comprehensive system status monitoring feature to the BFMC autonomous vehicle dashboard, answering the question "kako ide" (how is it going).

## Features Implemented

### Backend (Python)
- **New Message Type**: `SystemStatus` (ID: 18) in `allMessages.py`
- **Status Collection**: `getSystemStatus()` method in `processDashboard.py` that monitors:
  - System uptime and current time
  - Process activity and health
  - CPU, memory, and temperature metrics
  - Overall system status assessment
- **Real-time Broadcasting**: Status data sent via WebSocket every second

### Frontend (Angular)
- **New Component**: `KakoIdeComponent` with Serbian/Croatian interface
- **Real-time Updates**: WebSocket subscription for live status monitoring
- **Visual Indicators**: Color-coded status with warning thresholds
- **Responsive Design**: Matches existing dashboard styling

## File Changes

### Python Backend
- `src/utils/messages/allMessages.py` - Added SystemStatus message type
- `src/dashboard/processDashboard.py` - Added status collection and broadcasting

### Angular Frontend
- `src/dashboard/frontend/src/app/webSocket/web-socket.service.ts` - Added SystemStatus subscription
- `src/dashboard/frontend/src/app/cluster/cluster.component.*` - Integrated new component
- `src/dashboard/frontend/src/app/cluster/kako-ide/` - New component directory with:
  - `kako-ide.component.ts` - Component logic
  - `kako-ide.component.html` - Template with Serbian/Croatian text
  - `kako-ide.component.css` - Styling matching dashboard theme
  - `kako-ide.component.spec.ts` - Test specification

## Data Structure

The system status includes:
```json
{
  "timestamp": "2025-07-30 12:03:15",
  "uptime_seconds": 3672,
  "overall_status": "dobro",  // "dobro" = good, "paznja" = attention
  "system_health": {
    "cpu_ok": true,
    "memory_ok": true, 
    "temperature_ok": true
  },
  "active_processes": 8,
  "total_processes": 10,
  "message_activity": {...},
  "performance": {
    "cpu_usage": 45.2,
    "memory_usage": 62.8,
    "temperature": 55
  }
}
```

## Usage

1. **Backend**: Status is automatically collected and broadcast when dashboard process runs
2. **Frontend**: Component displays live status in the cluster dashboard
3. **Demo**: Run `python3 demo_kako_ide.py` to see sample output

## Health Assessment

The system evaluates health based on:
- CPU usage < 90%
- Memory usage < 85% 
- Temperature < 70°C

Status values:
- `"dobro"` - All systems healthy
- `"paznja"` - Attention needed (warnings present)

## Integration

The component is positioned in the bottom-right of the cluster dashboard, complementing the existing hardware-data component on the bottom-left.

## Testing

Run the demonstration script to verify the implementation:
```bash
python3 demo_kako_ide.py
```

This shows the message type integration and sample data structure that would be displayed in the dashboard.