function [Ec]=Energyfun(alpha,beta,dist1,dist2,A)
   
    
    switch(A)
        case 1
        %% Direct
            Ec=beta.*dist2;          
        case 2    
        %% LEACH
            Ec=alpha.*dist1 + beta.*dist2;          
        case 3    
        %% Hopbyhop(custom)
            Ec=alpha.*dist1;
        otherwise
            Ec=0;
   end
         
end