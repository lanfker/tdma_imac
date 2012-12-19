ber = 0;
gamma = 0:0.001:10;
for k = 2:16
    ber = ber + (-1)^k * (factorial(16)/(factorial(k)*factorial(16-k))) * exp(20*gamma*(1/k -1));
end
    ber = ber/30;
    
pdr = power ((1-ber),(8*132*8));

plot( gamma, pdr)

