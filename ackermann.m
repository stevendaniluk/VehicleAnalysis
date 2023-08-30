theta_wheel = linspace(0, 180);
tau_wheel = 15.8;
l = 2.3;
b = 1.457;

ack_vals = 0:0.2:1.0;


theta_ack = theta_wheel / tau_wheel;
theta_o_ack = atand(l * tand(theta_ack) ./ (l + 0.5 * b * tand(theta_ack)));
theta_i_ack = atand(l * tand(theta_ack) ./ (l - 0.5 * b * tand(theta_ack)));
for i=1:length(ack_vals)
    ack = ack_vals(i);
    eps = 0.5 * (1 - ack) * (theta_o_ack - theta_i_ack);

    theta_i = theta_i_ack + eps;
    theta_o = theta_o_ack - eps;

    theta_i_diff = theta_i - theta_i_ack;
    theta_o_diff = theta_o - theta_o_ack;

    plot(theta_ack, theta_i_diff, '-', 'DisplayName', sprintf('Inner - A=%.1f', ack));
    plot(theta_ack, theta_o_diff, '--', 'DisplayName', sprintf('Outer - A=%.1f', ack));
end


xlabel('Ackermann Angle [deg]');
ylabel('Wheel Difference [deg]');
legend(gca, 'show');
grid on;
