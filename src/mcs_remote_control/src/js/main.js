import ROSLIB from "roslib";
import EventEmitter2 from "eventemitter2";
import CONFIG from "./config.js";
import { auth } from "./auth.js";

class RobotController {
  constructor() {
    this.ros = null;
    this.cmdVelTopic = null;
    this.isConnected = false;
    this.speedMultiplier = CONFIG.CONTROLS.DEFAULT_SPEED;
    this.eventEmitter = new EventEmitter2();

    // Check authentication first
    if (!this.checkAuthentication()) {
      return;
    }

    this.init();
  }

  checkAuthentication() {
    if (!auth.isLoggedIn()) {
      // Redirect to login page if not authenticated
      window.location.href = CONFIG.AUTH.LOGIN_PAGE;
      return false;
    }
    return true;
  }

  init() {
    this.setupUI();
    this.setupEventListeners();
    this.connectToROS();
    this.updateSessionInfo();
  }

  setupUI() {
    // Get DOM elements
    this.elements = {
      statusIndicator: document.getElementById("statusIndicator"),
      statusText: document.getElementById("statusText"),
      speedSlider: document.getElementById("speedSlider"),
      speedValue: document.getElementById("speedValue"),
      controlButtons: document.querySelectorAll(".control-btn"),
      wsUrl: document.getElementById("wsUrl"),
      logoutBtn: document.getElementById("logoutBtn"),
      sessionStatus: document.getElementById("sessionStatus"),
      sessionExpires: document.getElementById("sessionExpires"),
    };

    // Set initial speed display
    this.updateSpeedDisplay();

    // Update WebSocket URL display
    if (this.elements.wsUrl) {
      this.elements.wsUrl.textContent = CONFIG.ROS.WEBSOCKET_URL;
    }
  }

  setupEventListeners() {
    // Speed control
    this.elements.speedSlider.addEventListener("input", (e) => {
      this.speedMultiplier = e.target.value / 100;
      this.updateSpeedDisplay();
    });

    // Control buttons
    this.elements.controlButtons.forEach((btn) => {
      btn.addEventListener("click", (e) => {
        const action = e.currentTarget.dataset.action;
        this.handleControlAction(action);
      });

      // Keyboard support
      btn.addEventListener("keydown", (e) => {
        if (e.key === "Enter" || e.key === " ") {
          e.preventDefault();
          const action = e.currentTarget.dataset.action;
          this.handleControlAction(action);
        }
      });
    });

    // Logout button
    if (this.elements.logoutBtn) {
      this.elements.logoutBtn.addEventListener("click", () => {
        this.handleLogout();
      });
    }

    // Keyboard controls
    document.addEventListener("keydown", (e) => {
      this.handleKeyboardInput(e);
    });

    document.addEventListener("keyup", (e) => {
      this.handleKeyboardRelease(e);
    });

    // Check authentication periodically
    setInterval(() => {
      if (!auth.isLoggedIn()) {
        this.handleLogout();
      }
    }, CONFIG.AUTH.AUTH_CHECK_INTERVAL);
  }

  handleLogout() {
    // Stop any ongoing robot movement
    this.publishCmdVel(0, 0);

    // Logout from auth system
    auth.logout();

    // Redirect to login page
    window.location.href = CONFIG.AUTH.LOGIN_PAGE;
  }

  updateSessionInfo() {
    const sessionInfo = auth.getSessionInfo();
    if (
      sessionInfo &&
      this.elements.sessionStatus &&
      this.elements.sessionExpires
    ) {
      this.elements.sessionStatus.textContent = "Authenticated";

      // Calculate time until expiration
      const now = Date.now();
      const expiresIn = sessionInfo.expires - now;
      const hours = Math.floor(expiresIn / (1000 * 60 * 60));
      const minutes = Math.floor((expiresIn % (1000 * 60 * 60)) / (1000 * 60));

      if (hours > 0) {
        this.elements.sessionExpires.textContent = `${hours}h ${minutes}m`;
      } else {
        this.elements.sessionExpires.textContent = `${minutes}m`;
      }
    }
  }

  connectToROS() {
    try {
      this.ros = new ROSLIB.Ros({ url: CONFIG.ROS.WEBSOCKET_URL });

      this.ros.on("connection", () => {
        if (CONFIG.DEV.DEBUG_MODE) {
          console.log("Connected to ROS2 WebSocket server");
        }
        this.isConnected = true;
        this.updateConnectionStatus("connected", "Connected");
        this.setupROSTopics();
        this.enableControls();
      });

      this.ros.on("error", (error) => {
        console.error("ROS2 connection error:", error);
        this.isConnected = false;
        this.updateConnectionStatus("disconnected", "Connection Error");
        this.disableControls();
      });

      this.ros.on("close", () => {
        if (CONFIG.DEV.DEBUG_MODE) {
          console.log("ROS2 connection closed");
        }
        this.isConnected = false;
        this.updateConnectionStatus("disconnected", "Disconnected");
        this.disableControls();

        // Attempt to reconnect if auto-reconnect is enabled
        if (CONFIG.DEV.AUTO_RECONNECT) {
          setTimeout(() => {
            if (!this.isConnected) {
              if (CONFIG.DEV.DEBUG_MODE) {
                console.log("Attempting to reconnect...");
              }
              this.connectToROS();
            }
          }, CONFIG.ROS.RECONNECT_INTERVAL);
        }
      });
    } catch (error) {
      console.error("Failed to initialize ROS2 connection:", error);
      this.updateConnectionStatus("disconnected", "Failed to Connect");
    }
  }

  setupROSTopics() {
    this.cmdVelTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: CONFIG.ROS.TOPIC_NAME,
      messageType: CONFIG.ROS.MESSAGE_TYPE,
    });
  }

  updateConnectionStatus(status, text) {
    const indicator = this.elements.statusIndicator;
    const statusText = this.elements.statusText;

    // Remove all status classes
    indicator.classList.remove("connected", "disconnected");

    // Add appropriate class
    indicator.classList.add(status);
    statusText.textContent = text;
  }

  updateSpeedDisplay() {
    const percentage = Math.round(this.speedMultiplier * 100);
    this.elements.speedValue.textContent = `${percentage}%`;
  }

  enableControls() {
    this.elements.controlButtons.forEach((btn) => {
      btn.disabled = false;
    });
  }

  disableControls() {
    this.elements.controlButtons.forEach((btn) => {
      btn.disabled = true;
    });
  }

  handleControlAction(action) {
    if (!this.isConnected || !this.cmdVelTopic) {
      console.warn("Not connected to ROS2");
      return;
    }

    let linearX = 0;
    let angularZ = 0;

    switch (action) {
      case "forward":
        linearX = CONFIG.CONTROLS.MAX_LINEAR_SPEED * this.speedMultiplier;
        break;
      case "backward":
        linearX = -CONFIG.CONTROLS.MAX_LINEAR_SPEED * this.speedMultiplier;
        break;
      case "left":
        angularZ = CONFIG.CONTROLS.MAX_ANGULAR_SPEED * this.speedMultiplier;
        break;
      case "right":
        angularZ = -CONFIG.CONTROLS.MAX_ANGULAR_SPEED * this.speedMultiplier;
        break;
      case "stop":
        linearX = 0;
        angularZ = 0;
        break;
      default:
        console.warn("Unknown action:", action);
        return;
    }

    this.publishCmdVel(linearX, angularZ);
  }

  handleKeyboardInput(e) {
    if (!this.isConnected) return;

    let action = null;
    const key = e.key.toLowerCase();

    if (CONFIG.KEYBOARD.FORWARD.includes(key)) {
      action = "forward";
    } else if (CONFIG.KEYBOARD.BACKWARD.includes(key)) {
      action = "backward";
    } else if (CONFIG.KEYBOARD.LEFT.includes(key)) {
      action = "left";
    } else if (CONFIG.KEYBOARD.RIGHT.includes(key)) {
      action = "right";
    } else if (CONFIG.KEYBOARD.STOP.includes(key)) {
      action = "stop";
    }

    if (action) {
      e.preventDefault();
      this.handleControlAction(action);
    }
  }

  handleKeyboardRelease(e) {
    if (!this.isConnected) return;

    // Stop on key release for movement keys
    const movementKeys = [
      ...CONFIG.KEYBOARD.FORWARD,
      ...CONFIG.KEYBOARD.BACKWARD,
      ...CONFIG.KEYBOARD.LEFT,
      ...CONFIG.KEYBOARD.RIGHT,
    ];

    if (movementKeys.includes(e.key.toLowerCase())) {
      this.handleControlAction("stop");
    }
  }

  publishCmdVel(linearX, angularZ) {
    if (!this.cmdVelTopic) {
      console.error("cmd_vel topic not initialized");
      return;
    }

    const twist = new ROSLIB.Message({
      linear: { x: linearX, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angularZ },
    });

    try {
      this.cmdVelTopic.publish(twist);
      if (CONFIG.DEV.DEBUG_MODE) {
        console.log(
          `Published cmd_vel: linear.x=${linearX.toFixed(
            2
          )}, angular.z=${angularZ.toFixed(2)}`
        );
      }
    } catch (error) {
      console.error("Failed to publish cmd_vel:", error);
    }
  }

  // Public methods for external access
  getConnectionStatus() {
    return this.isConnected;
  }

  getSpeedMultiplier() {
    return this.speedMultiplier;
  }

  setSpeedMultiplier(speed) {
    this.speedMultiplier = Math.max(0.1, Math.min(1.0, speed));
    this.updateSpeedDisplay();
  }
}

// Initialize the controller when the DOM is loaded
document.addEventListener("DOMContentLoaded", () => {
  if (CONFIG.DEV.DEBUG_MODE) {
    console.log("Initializing MCS Robot Controller...");
  }
  window.robotController = new RobotController();
});

// Export for potential module usage
export default RobotController;
