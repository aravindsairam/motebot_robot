# One straight piece of a cleaning plan, in the map frame (metres).
uint8 ROW = 0          # a cleaning pass
uint8 CONNECTOR = 1    # the short hop to the next row
uint8 TRANSIT = 2      # a trip Nav2 plans; only its ends are known in advance
uint8 EDGE = 3         # a pass along a wall or furniture

uint8 kind
float64 x0
float64 y0
float64 x1
float64 y1
uint32 number          # rows and edges: their number, from 1; 0 for the others
uint8 pass_number      # 1 or 2 for rows and the connectors between them
uint32 region          # rows of one region form one zig-zag
