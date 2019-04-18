function pidsearch_calibration(G,type)
switch(type)
    %% Using pidtuner to generate a PI controller
    case 'PI';
    % Using pidtuner to generate a PI controller
    C_pi = pidtune(G, 'PI');

    % Using pidsearch to generate a calibrate version of PI controller
    C_pi = pidsearch(G,C_pi,'OS')

    % Baseline controller effort and system transfer functions
    % Baseline System Transfer Function
    T_pi = (C_pi*G)/(1+(G*C_pi));

    %% Plot the Baseline vs. tuned system Step Response
    figure(3);
    hold on;
    subplot(1,2,1);
    
    step(T_pi)
    grid on;

    legend('Baseline');
    title('System Step Response of PI Contoller');
    hold off;

    % Plot the Baseline vs. tuned system Contoller Effort Step Response

    Tu = T_pi/G;
    subplot(1,2,2);
    step(Tu)
    grid on;

    legend('Baseline');
    title('Contoller Effort Step Response of PI');
    hold off;

    %% Using pidtuner to generate a PIDF controller
    case 'PIDF'

    % Using pidtuner to generate a PIDF controller
    C_pidf = pidtune(G, 'PIDF');

    % Using pidsearch to generate a calibrate version of PI controller
    C_pidf = pidsearch(G,C_pidf,'OS')

    % Baseline controller effort and system transfer functions
    % Baseline System Transfer Function
    T_pidf = (C_pidf*G)/(1+(G*C_pidf));

    %% Plot the Baseline vs. tuned system Step Response
    figure(4);
    hold on;
    subplot(1,2,1);
    
    step(T_pidf)
    grid on;

    legend('Baseline');
    title('System Step Response of PIDF Contoller');
    
    % Plot the Baseline vs. tuned system Contoller Effort Step Response
    hold on;
    subplot(1,2,2);
    
    Tu = T_pidf/G;
    step(Tu)
    grid on;

    legend('Baseline');
    title('Contoller Effort Step Response of PIDF');
    hold off;

    %% Using pidtuner to generate a PDF controller
    case 'PDF';

    % Using pidtuner to generate a PDF controller
    C_pdf = pidtune(G, 'PDF');

    % Using pidsearch to generate a calibrate version of PDF controller
    C_pdf = pidsearch(G,C_pdf,'OS')

    % Baseline controller effort and system transfer functions
    % Baseline System Transfer Function
    T_pdf = (C_pdf*G)/(1+(G*C_pdf));

    %% Plot the Baseline vs. tuned system Step Response
    figure(6);
    hold on;
    subplot(1,2,1);
    
    step(T_pdf)
    grid on;

    legend('Baseline');
    title('System Step Response of PDF Contoller');
    hold off;

    % Plot the Baseline vs. tuned system Contoller Effort Step Response
    hold on;
    subplot(1,2,2);
    
    Tu = T_pdf/G;
    step(Tu)
    grid on;

    legend('Baseline');
    title('Contoller Effort Step Response of PDF');
    hold off;
end