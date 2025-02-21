%% WEEK 2 - TASK 2 - Sensor Uncertainty
function [public_vars] = sensor_analysis(read_only_vars, public_vars)

    public_vars.lidar_history = [public_vars.lidar_history; read_only_vars.lidar_distances];
    public_vars.gnss_history = [public_vars.gnss_history; read_only_vars.gnss_position];
    
    if size(public_vars.lidar_history, 1) == 100  
        % Compute and display standard deviations for LˇIDAR and GNSS
        lidar_sigma = std(public_vars.lidar_history);     % 1x8 
        gnss_sigma = std(public_vars.gnss_history);       % 1x2
    
        disp('LiDAR Standard Deviations (for each channel):');
        disp(lidar_sigma);
        disp('GNSS Standard Deviations (x and y axes):');
        disp(gnss_sigma);
        
        % % Save data to file
        % lidar_file = 'lidar_data.csv';
        % gnss_file = 'gnss_data.csv';
        % 
        % % Write LiDAR and GNSS stds to CSV
        % writematrix(lidar_sigma, lidar_file);
        % disp(['LiDAR data saved to ', lidar_file]);
        % 
        % writematrix(gnss_sigma, gnss_file);
        % disp(['GNSS data saved to ', gnss_file]);

        % Histogram Plots LiDAR
        figure;
        for i = 1:8
            subplot(4, 2, i);
            histogram(public_vars.lidar_history(:, i), 20);
            title(['LiDAR Channel ', num2str(i)]);
            xlabel('Distance (m)');
            ylabel('Frequency');
        end
    
        % Histogram Plots GNSS
        figure;
        for i = 1:2
            subplot(2, 1, i);
            histogram(public_vars.gnss_history(:, i), 20);
            title(['GNSS Axis ', char('X' + i - 1)]);
            xlabel('Position (m)');
            ylabel('Frequency');
        end
        
        %% WEEK 2 - TASK 3 - Covariance matrix

        lidar_cov = cov(public_vars.lidar_history);
        gnss_cov = cov(public_vars.gnss_history);

        disp('LiDAR Covariance matrix (8x8):');
        disp(lidar_cov);
        disp('GNSS Covariance matrix (2x2):');
        disp(gnss_cov);

        % Comparison of LiDAR elements 
        lidar_sigma_squared = lidar_sigma.^2;
        lidar_diagonal_elements = diag(lidar_cov);

        disp('LiDAR Diagonal elements of the covariance matrix:');
        disp(lidar_diagonal_elements);
        disp('LiDAR Sigma^2 (variance):');
        disp(lidar_sigma_squared);

        % Comparison of GNSS elements 
        gnss_sigma_squared = gnss_sigma.^2;
        gnss_diagonal_elements = diag(gnss_cov);

        disp('GNSS Diagonal elements of the covariance matrix:');
        disp(gnss_diagonal_elements);
        disp('GNSS Sigma^2 (variance):');
        disp(gnss_sigma_squared);


        %% WEEK 2 - TASK 4 - Normal Distribution 
        max_sigma = max(lidar_sigma(1), gnss_sigma(1));
        x = linspace(-3*max_sigma, 3*max_sigma, 1000);
        mu = 0;

        lidar_norm_pdf = norm_pdf(x,mu,lidar_sigma(1));
        gnss_norm_pdf = norm_pdf(x,mu,gnss_sigma(1));

        figure;
        plot(x, lidar_norm_pdf, 'r', 'LineWidth', 2);
        hold on;
        plot(x, gnss_norm_pdf, 'b', 'LineWidth', 2);
        title('Normal Distribution PDFs for Sensor Noise (mu = 0)');
        xlabel('x');
        ylabel('Probability Density');
        legend('1st LiDAR Ch', 'GNSS X Axis');
        grid on;
        

        % Prevent further growth of data arrays to avoid memory issues
        public_vars.lidar_history = public_vars.lidar_history(end-99:end, :);
        public_vars.gnss_history = public_vars.gnss_history(end-99:end, :);

    end
end
