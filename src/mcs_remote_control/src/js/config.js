// MCS Remote Control Configuration
// Modify these settings to customize the application behavior

export const CONFIG = {
  // ROS2 WebSocket connection
  ROS: {
    WEBSOCKET_URL: "ws://localhost:9090",
    RECONNECT_INTERVAL: 5000, // milliseconds
    TOPIC_NAME: "/cmd_vel",
    MESSAGE_TYPE: "geometry_msgs/Twist",
  },

  // Control settings
  CONTROLS: {
    DEFAULT_SPEED: 0.5, // 50%
    MAX_LINEAR_SPEED: 0.5, // m/s
    MAX_ANGULAR_SPEED: 0.5, // rad/s
    SPEED_STEP: 0.1, // 10% increments
  },

  // UI settings
  UI: {
    BUTTON_SIZE: 100, // pixels
    ANIMATION_DURATION: 200, // milliseconds
    STATUS_UPDATE_INTERVAL: 1000, // milliseconds
  },

  // Keyboard mappings
  KEYBOARD: {
    FORWARD: ["w", "arrowup"],
    BACKWARD: ["s", "arrowdown"],
    LEFT: ["a", "arrowleft"],
    RIGHT: ["d", "arrowright"],
    STOP: [" "], // spacebar
  },

  // Authentication settings
  AUTH: {
    DEFAULT_PASSWORD: "robot2024", // Change this in production!
    SESSION_DURATION: 24 * 60 * 60 * 1000, // 24 hours in milliseconds
    AUTH_CHECK_INTERVAL: 30000, // Check auth every 30 seconds
    LOGIN_PAGE: "./login.html",
    MAIN_PAGE: "./index.html",
  },

  // Development settings
  DEV: {
    DEBUG_MODE: true,
    LOG_LEVEL: "info", // 'debug', 'info', 'warn', 'error'
    AUTO_RECONNECT: true,
  },
};

// Helper function to get configuration value with fallback
export function getConfig(path, defaultValue = null) {
  const keys = path.split(".");
  let value = CONFIG;

  for (const key of keys) {
    if (value && typeof value === "object" && key in value) {
      value = value[key];
    } else {
      return defaultValue;
    }
  }

  return value;
}

// Helper function to set configuration value
export function setConfig(path, value) {
  const keys = path.split(".");
  const lastKey = keys.pop();
  let current = CONFIG;

  for (const key of keys) {
    if (!(key in current) || typeof current[key] !== "object") {
      current[key] = {};
    }
    current = current[key];
  }

  current[lastKey] = value;
}

export default CONFIG;
