function [tbot] = connectRobot(connectionType)
    if ( ~exist("tbot") ) 
    
        switch connectionType
            case "sim"
                 IP_TURTLEBOT = "192.168.56.101";
                IP_HOST_COMPUTER = "192.168.56.1";
            case "tbot1"
                IP_TURTLEBOT = "192.168.1.201";
                IP_HOST_COMPUTER = "192.168.1.108";
            case "tbot2"
                IP_TURTLEBOT = "192.168.1.201";
                IP_HOST_COMPUTER = "192.168.1.108";
        end
    
        tbot = TurtleBot3(IP_TURTLEBOT, IP_HOST_COMPUTER);
    
        if( tbot.getVersion() < 0.9 ) 
            error('TurtleBot v09 required');
        end 
    end
end