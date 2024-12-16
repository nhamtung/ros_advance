document.addEventListener("DOMContentLoaded", function () {
  const loginForm = document.getElementById("login-form");
  const usernameInput = document.getElementById("username");
  const passwordInput = document.getElementById("password");
  const rememberMeCheckbox = document.getElementById("remember-me");
  const redirectSelect = document.getElementById("redirect");

  loginForm.addEventListener("submit", function (e) {
      e.preventDefault();

      const username = usernameInput.value;
      const password = passwordInput.value;
      const rememberMe = rememberMeCheckbox.checked;
      const redirectPage = redirectSelect.value;

      // Lưu thông tin đăng nhập nếu người dùng chọn "Lưu Mật Khẩu"
      if (rememberMe) {
          localStorage.setItem("username", username);
          localStorage.setItem("password", password);
      } else {
          localStorage.removeItem("username");
          localStorage.removeItem("password");
      }

      // Xác thực đăng nhập
      if (username === "admin" && password === "123456") {
          console.info("Đăng nhập thành công!");
          window.location.href = redirectPage; // Điều hướng đến trang được chọn
      } else {
          alert("Tài khoản hoặc mật khẩu không đúng!");
          console.info("Tài khoản hoặc mật khẩu không đúng!");
      }
  });

  // Hiển thị hoặc ẩn mật khẩu
  const showPasswordCheckbox = document.getElementById("show-password");
  showPasswordCheckbox.addEventListener("change", function () {
      passwordInput.type = showPasswordCheckbox.checked ? "text" : "password";
  });
});
