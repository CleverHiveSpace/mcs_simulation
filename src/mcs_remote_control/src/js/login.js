import { auth } from "./auth.js";
import CONFIG from "./config.js";

class LoginManager {
  constructor() {
    this.form = document.getElementById("loginForm");
    this.passwordInput = document.getElementById("password");
    this.loginBtn = document.getElementById("loginBtn");
    this.loginBtnText = document.getElementById("loginBtnText");
    this.loginBtnLoading = document.getElementById("loginBtnLoading");
    this.errorMessage = document.getElementById("errorMessage");
    this.successMessage = document.getElementById("successMessage");

    this.setupEventListeners();
    this.checkExistingSession();
  }

  setupEventListeners() {
    // Handle form submission
    this.form.addEventListener("submit", (e) => {
      e.preventDefault();
      this.handleLogin();
    });

    // Handle Enter key in password field
    this.passwordInput.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        this.handleLogin();
      }
    });

    // Clear error message when user starts typing
    this.passwordInput.addEventListener("input", () => {
      this.hideError();
    });
  }

  checkExistingSession() {
    // If user is already logged in, redirect to main app
    if (auth.isLoggedIn()) {
      this.redirectToMain();
    }
  }

  async handleLogin() {
    const password = this.passwordInput.value.trim();

    if (!password) {
      this.showError("Please enter a password");
      return;
    }

    // Show loading state
    this.setLoadingState(true);
    this.hideError();

    // Simulate a small delay for better UX
    await new Promise((resolve) => setTimeout(resolve, 500));

    try {
      // Attempt login
      const success = auth.login(password);

      if (success) {
        this.showSuccess();

        // Redirect after a short delay
        setTimeout(() => {
          this.redirectToMain();
        }, 1000);
      } else {
        this.showError("Invalid password. Please try again.");
        this.passwordInput.value = "";
        this.passwordInput.focus();
      }
    } catch (error) {
      console.error("Login error:", error);
      this.showError("An error occurred during login. Please try again.");
    } finally {
      this.setLoadingState(false);
    }
  }

  setLoadingState(loading) {
    if (loading) {
      this.loginBtn.disabled = true;
      this.loginBtnText.style.display = "none";
      this.loginBtnLoading.style.display = "inline";
    } else {
      this.loginBtn.disabled = false;
      this.loginBtnText.style.display = "inline";
      this.loginBtnLoading.style.display = "none";
    }
  }

  showError(message) {
    this.errorMessage.textContent = message;
    this.errorMessage.style.display = "block";
    this.successMessage.style.display = "none";
  }

  showSuccess() {
    this.successMessage.style.display = "block";
    this.errorMessage.style.display = "none";
  }

  hideError() {
    this.errorMessage.style.display = "none";
  }

  redirectToMain() {
    // Redirect to the main application
    window.location.href = CONFIG.AUTH.MAIN_PAGE;
  }
}

// Initialize login manager when DOM is loaded
document.addEventListener("DOMContentLoaded", () => {
  new LoginManager();
});
