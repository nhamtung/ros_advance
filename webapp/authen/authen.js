document.addEventListener("DOMContentLoaded", function () {
    const loginForm = document.getElementById("login-form");
    const usernameInput = document.getElementById("username");
    const passwordInput = document.getElementById("password");
    const rememberMeCheckbox = document.getElementById("remember-me");

    loginForm.addEventListener("submit", function (e) {
        e.preventDefault();

        const username = usernameInput.value;
        const password = passwordInput.value;
        const rememberMe = rememberMeCheckbox.checked;

        if (rememberMe) {
            localStorage.setItem("username", username);
            localStorage.setItem("password", password);
        } else {
            localStorage.removeItem("username");
            localStorage.removeItem("password");
        }

        // Xác thực đăng nhập
        if (username === "admin" && password === "123456") {
            console.info("dang nhap thanh cong");
            window.location.href = "../dynparam/index.html";
        } else {
            alert("Tài khoản hoặc mật khẩu không đúng!");
            console.info("Tài khoản hoặc mật khẩu không đúng!");
        }
    });
});