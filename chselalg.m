function [CH]=chselalg(EexL,nodesCH)
   
    
     %% LEACH
     [~,indE]=sort(EexL,'descend');
      CH=indE(1:nodesCH);


    
end