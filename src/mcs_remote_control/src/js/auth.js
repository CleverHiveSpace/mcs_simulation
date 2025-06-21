import CONFIG from "./config.js";

// Authentication module for MCS Remote Control
// Simple client-side authentication with session storage

export class AuthManager {
  constructor() {
    this.isAuthenticated = false;
    this.sessionKey = "mcs_remote_control_session";
    this.checkSession();
  }

  // Check if user has an active session
  checkSession() {
    const session = sessionStorage.getItem(this.sessionKey);
    if (session) {
      try {
        const sessionData = JSON.parse(session);
        const now = Date.now();

        // Check if session is still valid
        if (sessionData.expires > now) {
          this.isAuthenticated = true;
          return true;
        } else {
          // Session expired, clear it
          this.logout();
        }
      } catch (error) {
        console.error("Invalid session data:", error);
        this.logout();
      }
    }
    return false;
  }

  // Attempt to login with password
  login(password) {
    const correctPassword = CONFIG.AUTH.DEFAULT_PASSWORD;

    if (password === correctPassword) {
      this.isAuthenticated = true;

      // Create session data
      const sessionData = {
        authenticated: true,
        timestamp: Date.now(),
        expires: Date.now() + CONFIG.AUTH.SESSION_DURATION,
      };

      // Store session
      sessionStorage.setItem(this.sessionKey, JSON.stringify(sessionData));

      return true;
    }

    return false;
  }

  // Logout and clear session
  logout() {
    this.isAuthenticated = false;
    sessionStorage.removeItem(this.sessionKey);
  }

  // Check if user is authenticated
  isLoggedIn() {
    return this.isAuthenticated;
  }

  // Get session info
  getSessionInfo() {
    const session = sessionStorage.getItem(this.sessionKey);
    if (session) {
      try {
        return JSON.parse(session);
      } catch (error) {
        return null;
      }
    }
    return null;
  }

  // Change password (for future use)
  changePassword(newPassword) {
    // In a real application, this would be handled server-side
    // For now, we'll just update the config (not recommended for production)
    CONFIG.AUTH.DEFAULT_PASSWORD = newPassword;
    console.warn(
      "Password changed. In production, this should be handled server-side."
    );
  }
}

// Create global auth instance
export const auth = new AuthManager();
