// Kết nối tới ROS
const ros = new ROSLIB.Ros({
    url: 'ws://192.168.1.100:9090' // Thay <ROS_IP> bằng địa chỉ ROSBridge của bạn
});
ros.on('connection', () => {
    console.log('Connected to ROSBridge');
});
ros.on('error', (error) => {
    console.error('Error connecting to ROSBridge:', error);
});

// Tạo một đối tượng DynamicReconfigure để gửi giá trị tham số
const reconfigureService = new ROSLIB.Service({
    ros: ros,
    name: '/agv_dynparam/set_parameters', // Thay bằng dịch vụ dynamic_reconfigure của bạn
    serviceType: 'dynamic_reconfigure/Config'
});

function getCurrentBatteryLow() {
const param = new ROSLIB.Param({
    ros: ros,
    name: '/agv_dynparam/battery_low' // Đường dẫn đầy đủ đến tham số
});

param.get((value) => {
    if (value !== null && value !== undefined) {
    document.getElementById('current_battery_low').innerText = value;
    document.getElementById('battery_low').value = value;
    } else {
    console.error('Tham số battery_low không tồn tại hoặc không lấy được giá trị!');
    }
});
}

  // Update battery_low parameter
  function updateBatteryLow() {
    const value = parseInt(document.getElementById('battery_low').value, 20);

    const request = new ROSLIB.ServiceRequest({
      config: {
        ints: [
          {
            name: 'battery_low',
            value: value
          }
        ]
      }
    });

    reconfigureService.callService(request, (result) => {
      console.log('Battery Low updated:', result);
      getCurrentBatteryLow(); // Refresh displayed value
    });
  }

  // Fetch current value on page load
//   getCurrentBatteryLow();
  document.addEventListener('DOMContentLoaded', getCurrentBatteryLow);
