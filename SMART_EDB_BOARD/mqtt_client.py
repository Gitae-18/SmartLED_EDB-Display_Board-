"""Display gateway MQTT transport (Paho MQTT 2.x).

Example topic convention (adjust to the server's final protocol):
  display/{gateway_id}/command/#  server -> gateway
  display/{gateway_id}/response/... gateway -> server
  display/{gateway_id}/telemetry/... gateway -> server
  display/{gateway_id}/status gateway online/offline retained
"""
import json
import logging
import os
import ssl
import threading
import time
import traceback

import paho.mqtt.client as mqtt

try:
    import cbor2
except ImportError:
    cbor2 = None


class MqttBridge:
    def __init__(self, host, port=1883, base="display", client_id=None,
                 username=None, password=None, scheduler=None, router=None,
                 tls_ca=None, tls_cert=None, tls_key=None, net_iface=None):
        if not client_id:
            raise ValueError("client_id must be a stable, unique gateway ID")
        self.host = host
        self.port = int(port)
        self.base = base.strip("/")
        self.client_id = client_id
        self.gwid = client_id
        self.router = router
        self.scheduler = scheduler
        self.log = logging.getLogger("display.mqtt")
        self._handlers = []
        self._connected_event = threading.Event()
        self._stopping = False
        self._loop_started = False
        self._net_iface = net_iface or os.getenv("MQTT_NET_IFACE", "")
        self._keepalive = int(os.getenv("MQTT_KEEPALIVE_SEC", "60"))
        self._prefix = f"{self.base}/{self.gwid}"
        self._status_topic = f"{self._prefix}/status"
        self._command_topic = f"{self._prefix}/command/#"

        self._client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
            clean_session=True,
            protocol=mqtt.MQTTv311,
        )
        if username is not None:
            self._client.username_pw_set(username, password)
        if tls_ca or tls_cert or tls_key:
            if not tls_ca:
                raise ValueError("TLS requires a trusted CA certificate")
            self._client.tls_set(ca_certs=tls_ca, certfile=tls_cert,
                                 keyfile=tls_key, cert_reqs=ssl.CERT_REQUIRED,
                                 tls_version=ssl.PROTOCOL_TLS_CLIENT)
        self._client.will_set(self._status_topic,
                              self._encode(self._status_payload("offline", "lwt")),
                              qos=1, retain=True)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.on_publish = self._on_publish
        self._client.reconnect_delay_set(min_delay=1, max_delay=60)

    @staticmethod
    def _encode(obj):
        return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))

    def _status_payload(self, status, reason=None):
        data = {"cmd": "gateway_status", "gid": self.gwid,
                "status": status, "online": status == "online", "ts": int(time.time())}
        if reason:
            data["reason"] = reason
        return data

    def add_handler(self, fn):
        """Register fn(payload, topic). Called on MQTT network thread."""
        self._handlers.append(fn)

    def set_router(self, router):
        self.router = router

    def connect(self, host=None, port=None):
        if host:
            self.host = host
        if port is not None:
            self.port = int(port)
        self._stopping = False
        return self._client.connect(self.host, self.port, keepalive=self._keepalive)

    def loop_start(self):
        if not self._loop_started:
            self._loop_started = True
            self._client.loop_start()

    def loop_forever(self):
        self._client.loop_forever()

    def wait_connected(self, timeout=10):
        return self._connected_event.wait(timeout)

    def publish_json(self, topic, obj, qos=1, retain=False):
        info = self._client.publish(topic, self._encode(obj), qos=qos, retain=retain)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.log.warning("publish failed topic=%s rc=%s", topic, info.rc)
        return info

    def publish_cbor(self, topic, obj, qos=1):
        if cbor2 is None:
            raise RuntimeError("CBOR support requires: pip install cbor2")
        return self._client.publish(topic, cbor2.dumps(obj), qos=qos)

    def publish_gateway_status(self, status="online", reason=None):
        return self.publish_json(self._status_topic,
                                 self._status_payload(status, reason), retain=True)

    def publish_response(self, command, payload, qos=1):
        """Publish a command response; payload must contain application result fields."""
        return self.publish_json(f"{self._prefix}/response/{command}", payload, qos=qos)

    def publish_telemetry(self, category, payload, qos=1):
        return self.publish_json(f"{self._prefix}/telemetry/{category}", payload, qos=qos)

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self._connected_event.clear()
            self.log.error("MQTT connection rejected: %s", reason_code)
            return
        result, mid = client.subscribe(self._command_topic, qos=1)
        if result != mqtt.MQTT_ERR_SUCCESS:
            self.log.error("MQTT subscribe failed rc=%s", result)
            return
        self._connected_event.set()
        self.publish_gateway_status("online", "mqtt_connect")
        self.log.info("MQTT connected; subscribed %s mid=%s", self._command_topic, mid)

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        self._connected_event.clear()
        if not self._stopping:
            self.log.warning("MQTT disconnected: %s (Paho reconnect enabled)", reason_code)

    def _on_publish(self, client, userdata, mid, reason_code, properties):
        if reason_code.is_failure:
            self.log.warning("MQTT PUBACK failed mid=%s reason=%s", mid, reason_code)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeError, ValueError):
            if cbor2 is not None:
                try:
                    payload = cbor2.loads(msg.payload)
                except Exception:
                    payload = msg.payload
            else:
                payload = msg.payload
        for handler in tuple(self._handlers):
            try:
                handler(payload, msg.topic)
            except Exception:
                self.log.error("MQTT handler failed: %s", traceback.format_exc())

    def disconnect(self):
        self._stopping = True
        if self._connected_event.is_set():
            info = self.publish_gateway_status("offline", "shutdown")
            try:
                info.wait_for_publish(timeout=2)
            except (RuntimeError, ValueError):
                pass
        self._client.disconnect()
        self._connected_event.clear()
        if self._loop_started:
            self._client.loop_stop()
            self._loop_started = False

    def configure_tls_from(self, cert_dir="/etc/mosquitto/certs", gateway_id=None):
        """Call before connect(). Uses certificate verification, never insecure TLS."""
        gid = gateway_id or self.gwid
        self._client.tls_set(
            ca_certs=os.path.join(cert_dir, "ca.crt"),
            certfile=os.path.join(cert_dir, f"{gid}.crt"),
            keyfile=os.path.join(cert_dir, f"{gid}.key"),
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS_CLIENT,
        )

    def configure_tls_min12_from(self, cert_dir, gateway_id):
        """Backward-compatible alias; verifies the broker certificate."""
        self.configure_tls_from(cert_dir, gateway_id)
