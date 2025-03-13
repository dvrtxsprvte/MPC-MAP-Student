function [new_particles] = resample_particles(particles, weights)
    N = size(particles, 1);
    new_particles = zeros(size(particles));

    weights = weights ./ sum(weights);

    r = rand(1)/N;
    c = weights(1);
    i = 1;

    for m = 1:N
        U = r + (m-1)/N;
        while U > c
            i = i + 1;
            c = c + weights(i);
        end
        new_particles(m,:) = particles(i,:);
    end
end
