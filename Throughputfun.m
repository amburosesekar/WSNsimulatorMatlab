function [Throughput]=Throughputfun(Throughput,RxData1,ipp)
          
            if(ipp>1)
            Throughput=Throughput(ipp)+(RxData1);
            else
            Throughput=(RxData1);    
            end
end