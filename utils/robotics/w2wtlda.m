function out = w2wtlda(w)
    out = [w(1)^2, 2*w(1)*w(2), 2*w(1)*w(3),...
            w(2)^2, 2*w(2)*w(3), w(3)^2];
end