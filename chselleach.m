  function [Chleach]=chselleach(EexL,maxCHleach)
            EexL(EexL<=0)=0;
            indg3=find(EexL<=0)
            [~,indE]=sort(EexL,'descend');
            %maxCHleach=10;
            Chleach=indE(1:maxCHleach);
end