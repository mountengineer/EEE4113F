from flask import Flask, jsonify, request, send_file
from flask_sqlalchemy import SQLAlchemy
from flask_cors import CORS
from flask_socketio import SocketIO
from datetime import datetime
import binascii, struct, serial, threading, time, json, os

# ==========================================
# SERIAL BRIDGE CONFIG  ← change COM_PORT to match your HC-05
# ==========================================
COM_PORT  = "COM7"
BAUD_RATE = 9600

# ==========================================
# APP SETUP
# ==========================================
app = Flask(__name__)
CORS(app)
app.config['SQLALCHEMY_DATABASE_URI']        = 'sqlite:///buoy_data.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

db       = SQLAlchemy(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

ser = None   # global serial handle shared between threads

# ==========================================
# DATABASE MODELS
# ==========================================

class Buoy(db.Model):
    __tablename__ = 'buoys'
    node_id         = db.Column(db.String(5),  primary_key=True)
    current_mode    = db.Column(db.String(2),  nullable=False, default="01")
    date_registered = db.Column(db.DateTime,   default=datetime.utcnow)
    telemetry       = db.relationship('Telemetry', backref='buoy', lazy=True)

class Telemetry(db.Model):
    __tablename__ = 'telemetry'
    __table_args__ = (
        db.Index('idx_node_ts', 'node_id', db.desc('timestamp')),
    )
    id                      = db.Column(db.Integer,  primary_key=True)
    node_id                 = db.Column(db.String(5), db.ForeignKey('buoys.node_id'), nullable=False)
    timestamp               = db.Column(db.DateTime,  default=datetime.utcnow, nullable=False)
    latitude                = db.Column(db.Float,     nullable=True)
    longitude               = db.Column(db.Float,     nullable=True)
    significant_wave_height = db.Column(db.Float,     nullable=True)
    moments                 = db.Column(db.Float,     nullable=True)
    battery_percentage      = db.Column(db.Integer,   nullable=True)
    lora_rssi               = db.Column(db.Float,     nullable=True)
    lora_snr                = db.Column(db.Float,     nullable=True)

class PendingCommand(db.Model):
    __tablename__ = 'pending_commands'
    id        = db.Column(db.Integer,   primary_key=True)
    node_id   = db.Column(db.String(5),  db.ForeignKey('buoys.node_id'), nullable=False)
    command   = db.Column(db.String(20), nullable=False)
    param     = db.Column(db.String(10), nullable=False)
    queued_at = db.Column(db.DateTime,   default=datetime.utcnow)
    sent      = db.Column(db.Boolean,    default=False)

# ==========================================
# SERIAL BRIDGE  (HC-05 -> Socket.IO)
# ==========================================

def try_store_telemetry(payload):
    """Persist a valid telemetry JSON packet from the serial bridge into SQLite."""
    raw_id = payload.get('node_id') or payload.get('nodeID')
    if not raw_id:
        return
    node_id = str(raw_id).zfill(2)

    with app.app_context():
        buoy = db.session.get(Buoy, node_id)
        if not buoy:
            buoy = Buoy(node_id=node_id,
                        current_mode=str(payload.get('mode', '01')).zfill(2))
            db.session.add(buoy)
            db.session.flush()
            print(f"[bridge] Auto-registered buoy {node_id}")

        if payload.get('lat') is not None and payload.get('lon') is not None:
            rec = Telemetry(
                node_id                 = node_id,
                latitude                = payload.get('lat'),
                longitude               = payload.get('lon'),
                significant_wave_height = payload.get('Hs'),
                moments                 = payload.get('moments'),
                lora_rssi               = payload.get('rssi') or payload.get('loraRSSI'),
                lora_snr                = payload.get('snr')  or payload.get('loraSNR'),
            )
            db.session.add(rec)

        if payload.get('mode'):
            buoy.current_mode = str(payload['mode']).zfill(2)

        db.session.commit()


def serial_thread():
    """Background thread: keeps HC-05 serial port open and forwards lines to browser."""
    global ser
    rx_buf = ''

    while True:
        try:
            if ser is None:
                print(f"[bridge] Connecting to {COM_PORT} @ {BAUD_RATE} baud...")
                ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
                print(f"[bridge] Serial port open.")
                socketio.emit('bridge_status', {'connected': True, 'port': COM_PORT})

            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting).decode(errors='ignore')
                rx_buf += chunk

                while '\n' in rx_buf:
                    line, rx_buf = rx_buf.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    print(f"[bridge] RX: {line}")

                    parsed = None
                    try:
                        parsed = json.loads(line)
                    except Exception:
                        pass

                    socketio.emit('bt_message', {'message': line, 'parsed': parsed})

                    if parsed:
                        try:
                            try_store_telemetry(parsed)
                        except Exception as e:
                            print(f"[bridge] DB store error: {e}")

        except Exception as e:
            print(f"[bridge] Serial error: {e}")
            socketio.emit('bridge_status', {
                'connected': False, 'port': COM_PORT, 'error': str(e)
            })
            try:
                if ser:
                    ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(3)

# ==========================================
# SOCKET.IO EVENTS
# ==========================================

@socketio.on('connect')
def on_connect():
    connected = ser is not None and ser.is_open
    socketio.emit('bridge_status', {'connected': connected, 'port': COM_PORT})

# ==========================================
# ROOT — serve the dashboard HTML
# ==========================================

@app.route('/')
def index():
    here      = os.path.dirname(os.path.abspath(__file__))
    html_path = os.path.join(here, 'Buoy_Dashboard_v6.html')
    if not os.path.exists(html_path):
        return (
            "<h2 style='font-family:sans-serif;color:#c00'>Dashboard not found</h2>"
            "<p style='font-family:sans-serif'>Make sure <code>Buoy_Dashboard_v6.html</code> "
            "is in the same folder as <code>app.py</code>.</p>",
            404
        )
    return send_file(html_path)


@socketio.on('send_command')
def on_send_command(data):
    cmd = (data.get('cmd') or '').strip()
    if not cmd:
        return
    print(f"[bridge] TX cmd → {cmd}")
    try:
        if ser and ser.is_open:
            ser.write((cmd + '\n').encode())
        else:
            print("[bridge] Cannot send — serial port not open")
    except Exception as e:
        print(f"[bridge] Serial write error: {e}")
        
# ==========================================
# REST API — HEALTH CHECK
# ==========================================

@app.route('/api/status', methods=['GET'])
def api_status():
    bridge_ok = ser is not None and ser.is_open
    return jsonify({
        "status":            "ok",
        "buoys":             Buoy.query.count(),
        "telemetry_records": Telemetry.query.count(),
        "bridge_port":       COM_PORT,
        "bridge_connected":  bridge_ok,
    })

# ==========================================
# REST API — BUOY REGISTRATION
# ==========================================

@app.route('/api/buoys/add', methods=['POST'])
def add_buoy():
    data     = request.json
    new_id   = data.get('id')
    new_mode = data.get('mode', '01')
    if not new_id:
        return jsonify({"status": "error", "message": "Node ID required"}), 400
    if db.session.get(Buoy, new_id):
        return jsonify({"status": "error", "message": "Buoy already exists"}), 400
    db.session.add(Buoy(node_id=new_id, current_mode=new_mode))
    db.session.commit()
    return jsonify({"status": "success", "message": f"Buoy {new_id} registered."}), 201

# ==========================================
# REST API — TELEMETRY INGEST
# ==========================================

@app.route('/api/telemetry/add', methods=['POST'])
def add_telemetry():
    data    = request.json
    node_id = data.get('node_id')
    buoy    = db.session.get(Buoy, node_id)
    if not buoy:
        return jsonify({"status": "error", "message": "Buoy not found"}), 404
    rec = Telemetry(
        node_id                 = node_id,
        latitude                = data.get('lat'),
        longitude               = data.get('lon'),
        significant_wave_height = data.get('Hs'),
        moments                 = data.get('moments'),
        lora_rssi               = data.get('rssi'),
        lora_snr                = data.get('snr'),
    )
    db.session.add(rec)
    db.session.commit()
    return jsonify({"status": "success", "message": "Telemetry saved."}), 201

# ==========================================
# REST API — IRIDIUM SBD WEBHOOK
# ==========================================

@app.route('/api/webhook/iridium', methods=['POST'])
def iridium_webhook():
    data        = request.json if request.is_json else request.form
    hex_payload = data.get('data')
    if not hex_payload:
        return jsonify({"status": "error", "message": "No payload"}), 400
    try:
        raw      = binascii.unhexlify(hex_payload)
        unpacked = struct.unpack('< B f f B f I', raw)
        node_id  = str(unpacked[0]).zfill(2)
        lat, lon = unpacked[1], unpacked[2]
        mode     = str(unpacked[3]).zfill(2)
        hs       = unpacked[4]

        buoy = db.session.get(Buoy, node_id)
        if not buoy:
            buoy = Buoy(node_id=node_id, current_mode=mode)
            db.session.add(buoy)
        buoy.current_mode = mode
        db.session.add(Telemetry(node_id=node_id, latitude=lat,
                                 longitude=lon, significant_wave_height=hs))
        db.session.commit()
        return jsonify({"status": "success", "node_id": node_id}), 201
    except struct.error:
        return jsonify({"status": "error", "message": "Payload format mismatch"}), 500
    except Exception as e:
        print(f"[iridium] {e}")
        return jsonify({"status": "error", "message": "Internal error"}), 500

# ==========================================
# REST API — FLEET STATUS
# ==========================================

@app.route('/api/buoys', methods=['GET'])
def get_buoys():
    fleet = {}
    for b in Buoy.query.all():
        latest = (Telemetry.query.filter_by(node_id=b.node_id)
                  .order_by(Telemetry.timestamp.desc()).first())
        last_ms = int((latest.timestamp if latest else b.date_registered).timestamp() * 1000)
        fleet[b.node_id] = {
            "id":       b.node_id,
            "mode":     b.current_mode,
            "lat":      latest.latitude               if latest else None,
            "lon":      latest.longitude              if latest else None,
            "Hs":       latest.significant_wave_height if latest else None,
            "moments":  latest.moments                if latest else None,
            "battery":  None,
            "loraRSSI": latest.lora_rssi              if latest else None,
            "loraSNR":  latest.lora_snr               if latest else None,
            "lastSeen": last_ms,
            "gpsFix":   latest is not None and latest.latitude is not None,
        }
    return jsonify(fleet), 200

# ==========================================
# REST API — HISTORY
# ==========================================

@app.route('/api/history', methods=['GET'])
def get_history():
    history = {}
    for b in Buoy.query.all():
        records = list(reversed(
            Telemetry.query.filter_by(node_id=b.node_id)
            .order_by(Telemetry.timestamp.desc()).limit(500).all()
        ))
        history[b.node_id] = [
            {"ts":   int(r.timestamp.timestamp() * 1000),
             "lat":  r.latitude, "lon": r.longitude,
             "Hs":   r.significant_wave_height,
             "rssi": r.lora_rssi, "snr": r.lora_snr,
             "mode": b.current_mode}
            for r in records if r.latitude is not None
        ]
    return jsonify(history), 200

# ==========================================
# REST API — COMMAND QUEUE
# ==========================================

@app.route('/api/command', methods=['POST'])
def queue_command():
    data    = request.json
    node_id = data.get('node_id')
    command = data.get('command', 'SET_MODE')
    param   = data.get('param')
    if not node_id or not param:
        return jsonify({"status": "error", "message": "node_id and param required"}), 400
    buoy = db.session.get(Buoy, node_id)
    if not buoy:
        return jsonify({"status": "error", "message": "Buoy not found"}), 404
    if command == 'SET_MODE':
        buoy.current_mode = param
    db.session.add(PendingCommand(node_id=node_id, command=command, param=param))
    db.session.commit()
    return jsonify({"status": "queued", "node_id": node_id,
                    "command": command, "param": param,
                    "message": "Command queued."}), 201

@app.route('/api/commands/pending', methods=['GET'])
def get_pending_commands():
    cmds = (PendingCommand.query.filter_by(sent=False)
            .order_by(PendingCommand.queued_at.asc()).all())
    return jsonify([{"id": c.id, "node_id": c.node_id, "command": c.command,
                     "param": c.param, "queued_at": c.queued_at.isoformat()}
                    for c in cmds]), 200

# ==========================================
# STARTUP
# ==========================================

if __name__ == '__main__':
    with app.app_context():
        db.create_all()
        print("Database ready.")

    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()

    socketio.run(
        app,
        host="0.0.0.0",
        port=5000,
        allow_unsafe_werkzeug=True
    )