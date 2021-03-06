clc
lambda = 20;
rows = 5;
columns = 5;
unitWidth = 120;
poissonRandomNumbers= poissrnd(lambda, rows,columns)
retMatrix = [];
for ii = 1:inf
    

    for i = 1:rows
        for j = 1:columns % For every poisson random number indicating the total nodes in a region, generate a 2D coordinate for each node. 
            randomCoordinates = rand(poissonRandomNumbers(i,j), 2);
            scalerMatrix = [unitWidth 0; 0 unitWidth];
            for l = 1:poissonRandomNumbers(i,j) % the nodes position is mapped to a larger 2D plane.
                randomCoordinates(l,1) = randomCoordinates(l,1)*unitWidth + (i-1)*unitWidth;
                randomCoordinates(l,2) = randomCoordinates(l,2)*unitWidth + (j-1)*unitWidth;
            end
            retMatrix = [ retMatrix' randomCoordinates']';
        end
    end

    NodesCount = sum(sum(poissonRandomNumbers))
    DesiredLinkCount = 0;
    for it1 = 1:NodesCount
        for it2 = 1:NodesCount
           distance = sqrt ((retMatrix(it1,1)-retMatrix(it2,1))^2 + (retMatrix(it1,2)-retMatrix(it2,2))^2);
           if distance >= 44 && distance <=68
               distance
               DesiredLinkCount = DesiredLinkCount + 1;
               break;
           end
        end
    end

    if DesiredLinkCount/NodesCount >= 0.5
        fileId = fopen('data.txt', 'w');
        fprintf( fileId, '%2.6f %2.6f 0 \n', retMatrix');

        plot(retMatrix(:, 1), retMatrix(: , 2 ), '.')
        sum(sum(poissonRandomNumbers))
        break;
    end
end
