import json
from random import sample

moves = 0
move_Dict = {}
tot_distance = 0
all_best_balances = []

def balance(sampleJson):
    
    right_weight, left_weight, maxWeight, max_Index, isBalanced, total = getWeights(sampleJson)

    # Code to check if things are balanceable
    rest_of_weight = right_weight + left_weight - maxWeight
    # if maxWeight * .9 > rest_of_weight:
    #     print("Unbalanceable! IMPLEMENT SPECIAL CASE HERE")
    #     sift(sampleJson)
    #     return

    print("These are the initial weights. Left side total is: " + str(left_weight) + ". Right side total is: " + str(right_weight) + "." )
    if left_weight == 0 and right_weight == 0:
        balance_ratio = 1/1
    elif left_weight == 0:
        balance_ratio = 1/right_weight
    elif right_weight == 0:
        balance_ratio = left_weight/1
    else:
        balance_ratio = left_weight/right_weight
    best_balance = 100000.0 #This value will hold the ratio closest to balanced
    best_index = ""
    curr_balance = 100000.0 #This value will hold the ratio of the containers we are currently considering swapping.
    
    if isBalanced == True: #When ship is balanced
        print("The ship is already balanced!")
        
    while(balance_ratio > 1.1 or balance_ratio < .9):
        
        all_best_balances.append(best_balance)
        if len(all_best_balances) >= 2:
            if all_best_balances[-2] == all_best_balances[-1]:
                print("UNBALANCEABLE SHIP. EXECUTING SIFT")
                sift(sampleJson)
                return
                
        if balance_ratio > 1.1: #Left side is heavier we need to move something from here to right side
            
            for i in range(8,0,-1):
                for j in range(6,0,-1):
                    left_cont = makeIndex(i,j) #left_cont is just the index so we can access the weight values
                    new_left = left_weight - int(sampleJson[left_cont]["weight"])
                    new_right = right_weight + int(sampleJson[left_cont]["weight"])
                    if new_left == 0:
                        curr_balance = 1/new_right
                    elif new_right == 0:
                        curr_balance = new_left/1
                    else:
                        curr_balance = new_left/new_right
                    if curr_balance < 1.1 and curr_balance > 0.9:
                        #MOVE left_cont to nearest rightside spot CODE MUST BREAK HERE SINCE WE BALANCED THE SHIP
                        dest_y, dest_x = ifRightEmpty(sampleJson)
                        if dest_y != -1 and dest_x != -1:
                            move(i, j, dest_y, dest_x, sampleJson)
                            return #BREAK SINCE WE BALANCED THE CODE
                    else:
                        if abs(curr_balance-1) < abs(best_balance-1):
                            best_balance = curr_balance
                            best_y, best_x = i, j
            
            #If we made it here in our code that means that no move balanced the ship, so we will make the best move we can currently make.
            #MOVE best_index to the nearest rightside spot, and continue the while loop.
            dest_y, dest_x = ifRightEmpty(sampleJson)
            if dest_y != -1 and dest_x != -1:
                print("Left to Right:", best_y, best_x, dest_y, dest_x)
                move(best_y, best_x, dest_y, dest_x, sampleJson)
            
        if balance_ratio < .9: #Right side is heavier we need to move something from here to left side.
            
            for i in range(8,0,-1):
                for j in range(7,13):
                    right_cont = makeIndex(i,j)
                    new_right = right_weight - int(sampleJson[right_cont]["weight"])
                    new_left = left_weight + int(sampleJson[right_cont]["weight"])
                    if new_left == 0:
                        curr_balance = 1/new_right
                    elif new_right == 0:
                        curr_balance = new_left/1
                    else:
                        curr_balance = new_left/new_right
                    if curr_balance < 1.1 and curr_balance > 0.9:
                        #Move right_cont to nearest leftside spot CODE MUST BREAK HERE SINCE WE BALANACED THE SHIP
                        dest_y, dest_x = ifLeftEmpty(sampleJson)
                        if dest_y != -1 and dest_x != -1:
                            move(i, j, dest_y, dest_x, sampleJson)
                            return #BREAK SINCE WE BALANCED THE CODE
                    else:
                        if abs(curr_balance-1) < abs(best_balance-1):
                            best_balance = curr_balance
                            best_y, best_x = i,j
                            
            #If we made it here in our code that means that no move balanced the ship, so we will make the best move we can currently make.
            #Move best_index to the nearest leftside spot, and continue the while loop.
            dest_y, dest_x = ifLeftEmpty(sampleJson)
            if dest_y != -1 and dest_x != -1:
                print("Right to Left:", best_y, best_x, dest_y, dest_x)
                move(best_y, best_x, dest_y, dest_x, sampleJson)
            
        #After making some sort of change by either doing the right and left movement, we need to recalculate balance_ratio.
        left_weight = getLeftWeight(sampleJson)
        right_weight = getRightWeight(sampleJson)
        balance_ratio = left_weight/right_weight
        print("This is the total weight of the left side: " + str(left_weight) + ". This is the weight of the right side: " + str(right_weight) + ". The balance ratio is: " + str(balance_ratio) + ".")
        # print(left_weight, right_weight, balance_ratio)
#----------------------------------------------------------balance() ends here--------------------------------------------------------------#

def sift(sampleJson):                                                   # Sift is only done when the ship is not balanceable
    # moves = 0                                   
    # move_Dict = {}
    # tot_distance = 0
    sorted_array = []                                                   # To figure out which containers are heaviest to lightest
    for i in range(8):                                                  # Grabs all containers with weight to sort
        for j in range(12): 
            index = makeIndex(i+1,j+1)
            if sampleJson[index]["weight"] != "00000":
                entry = (int(sampleJson[index]["weight"]), sampleJson[index]["description"], i+1, j+1)
                sorted_array.append(entry)
    sorted_array.sort(key = lambda x: x[0], reverse=True)

    # Flag_x tells us if we need to place to the left or to the right, with help of column and rebound to go from 6 to 7, then to 5, then 8, etc, with a "rebounding" motion
    # When a column is completely filled, go up a row, and reset rebound to 0
    flag_x = 0
    column = 6
    row = 1
    rebound = 0

    for i in range(len(sorted_array)):                              # For all containers with weight in the manifest
        if checkDesc(row, column, sampleJson) != "UNUSED" and checkDesc(row, column, sampleJson) != "NAN": # CANNOT MOVE CONTAINER TO NAN 
            if checkDesc(row, column, sampleJson) != sorted_array[i][1]: # Don't move a container if it's already where it should be
                move_blocking_container_to_y = 0                         # This case handles moving to the optimal spot if there is another container in the way
                move_blocking_container_to_x = 0
                if flag_x == 0:
                    move_blocking_container_to_y, move_blocking_container_to_x = findFirstLeftCol(row, column, sampleJson)
                else: # if flag == 1
                    move_blocking_container_to_y, move_blocking_container_to_x = findFirstRightCol(row, column, sampleJson)
                move(row, column, move_blocking_container_to_y, move_blocking_container_to_x, sampleJson, 0)
                move(sorted_array[i][2], sorted_array[i][3], row, column, sampleJson, 0)
        elif (checkDesc(row, column, sampleJson) == "UNUSED"):          # If there is no container at optimal spot, no moveout needed
                move(sorted_array[i][2], sorted_array[i][3], row, column, sampleJson, 0)
        else:
            i -= 1

        sorted_array = []                                               # Resort array to update positions with potential changes
        for it in range(8):
            for j in range(12):
                index = makeIndex(it+1,j+1)
                if sampleJson[index]["weight"] != "00000":
                    entry = (int(sampleJson[index]["weight"]), sampleJson[index]["description"], it+1, j+1)
                    sorted_array.append(entry)
        sorted_array.sort(key = lambda x: (x[0], x[1]), reverse=True)

        if flag_x == 0:                                                 # First move to the right
            flag_x = 1
            column = column + rebound + 1
        else:
            flag_x = 0                                                  # Then move back to the left, one more position away from last container
            column = column - rebound - 1
        if column == 0 or column == 12:                                 # Reset rebound if column is filled, and go up a row
            column = 6
            row += 1
            rebound = 0
        rebound = rebound + 1
            
#----------------------------------------------------------Helper-Functions-----------------------------------------------------------------#
def makeIndex(y, x):
    if y > 9:   # If the index if greater than 9, then we format as a double digit instead of appending a 0 to the end
        if x > 9: # Same situation as y
            index = "["+ str(y) + "," + str(x) + "]"
        else:
            index = "["+ str(y) + ",0" + str(x) + "]"
    else:
        if x > 9:
            index = "[0"+ str(y) + "," + str(x) + "]"
        else:
            index = "[0"+ str(y) + ",0" + str(x) + "]"
    return index

def checkDesc(y, x, sampleJson): #This function will take the y and x as well as the Json we are operating on and check the description of the package
    index = makeIndex(y,x) 
    return sampleJson[index]["description"]

def ifLeftEmpty(sampleJson): # This function needs to be workshopped into finding the nearest empty space TO FIX
    for i in range(6,0,-1):
        for j in range(1,9):
            index = makeIndex(j,i)
            if checkDesc(j,i,sampleJson) == "UNUSED":
                return j,i
    return -1, -1

def ifRightEmpty(sampleJson): # This function needs to be workshopped into finding the nearest empty space TO FIX
    for i in range(7,13):
        for j in range(1,9):
            index = makeIndex(j,i)
            if checkDesc(j,i,sampleJson) == "UNUSED":
                return j, i
    return -1, -1

def getLeftWeight(sampleJson): #This function will return as a int the entire weight of the left hand side.
    total = 0
    for i in range(8): #Height
        for j in range(6):
            index = makeIndex(i+1,j+1)
            total += int(sampleJson[index]["weight"])
    return total

def getRightWeight(sampleJson): #This function will return as a int the entire weight of the right hand side.
    total = 0
    for i in range(8): #Height
        for j in range(6):
            index = makeIndex(i+1,j+7)
            total += int(sampleJson[index]["weight"])
            
    return total

def getWeights(sampleJson): #This function will return 5 values, int right_weight, int left_weight, int maxWeight, string max_Index, int total, and string heavy.
    total = 0
    maxWeight = 0
    max_Index = ""
    isBalanced = False
    right_weight = getRightWeight(sampleJson)
    left_weight = getLeftWeight(sampleJson)
    
    for i in range(8):
        for j in range(12):
            index = makeIndex(i+1,j+1)
            currWeight = int(sampleJson[index]["weight"])
            total += currWeight
            if currWeight > maxWeight:
                maxWeight = currWeight
                max_Index = index
                
    
    if left_weight == 0 and right_weight == 0:
        isBalanced = True
    elif left_weight == 0 or right_weight == 0:
        isBalanced = False
    elif left_weight/right_weight < 1.1 and left_weight/right_weight > .9:
        isBalanced = True
    
    return right_weight, left_weight, maxWeight, max_Index, isBalanced, total


#-----------------------------------------------------Move Functions---------------------------------------------------------------------#
            
def move(start_y, start_x, dest_y, dest_x, sampleJson, flag=0):
    
    clearPath(start_y, start_x, sampleJson) #This function will ensure we only move a container once all containers above it are gone
    
    if flag == 1: # This is for when we move containers to the nearest unused spot, since it can change while in clearPath, we must reassess what is the nearestUnused spot
        dest_y, dest_x = findNearestUnused(start_y, start_x, sampleJson)
        move(start_y, start_x, dest_y, dest_x, sampleJson)
        return
    
    start_index = makeIndex(start_y, start_x)
    goal_index = makeIndex(dest_y, dest_x)
    start_Weight = sampleJson[start_index]["weight"]
    start_Desc = checkDesc(start_y, start_x, sampleJson)
    
    distance = abs(start_y-dest_y) + abs(start_x-dest_x)
    global tot_distance
    tot_distance += distance
    global moves
    moves += 1
    global move_Dict
    move_Dict[str(moves)] = {}
    move_Dict[str(moves)]["origin"] = start_index
    move_Dict[str(moves)]["destination"] = goal_index
    
    #We only move to UNUSED containers, so by swapping it simulates the movement perfectly
    sampleJson[start_index]["weight"] = sampleJson[goal_index]["weight"]
    sampleJson[start_index]["description"] = sampleJson[goal_index]["description"]
    
    sampleJson[goal_index]["weight"] = start_Weight
    sampleJson[goal_index]["description"] = start_Desc
    

def clearPath(y, x, sampleJson):                                # Will move containers above our start container if nessessary-
    if y < 8:
        if checkDesc(y+1,x,sampleJson) != "UNUSED":       # URGENT: Check if y != 8 before checkDesc
            dest_y, dest_x = findNearestUnused(y+1, x, sampleJson)
            move(y+1, x, dest_y, dest_x, sampleJson, 1)

def findNearestUnused(y,x,sampleJson):                          # Finds nearest column to move blocking containers to
    if x < 7:                                                   # If container is on the left half, we ideally want to move it left
        if x == 1:                                              # Leftmost column can't move more left
            return findFirstRightCol(y, x, sampleJson)
        else:       
            dest_y, dest_x = findFirstLeftCol(y, x, sampleJson) # If no space on left, then search to the right
            if dest_y == -1 and dest_x == -1:
                return findFirstRightCol(y, x, sampleJson)
            else:
                return dest_y, dest_x
    else:                                                       # Containers on right half want to go more right
        if x == 12:                                             # Rightmost container can't move more right
            return findFirstLeftCol(y, x, sampleJson)           
        else:                                                   # If no space on right, then search to the left
            dest_y, dest_x = findFirstRightCol(y, x, sampleJson)
            if dest_y == -1 and dest_x == -1:
                return findFirstLeftCol(y, x, sampleJson)
            else:
                return dest_y, dest_x
            
def findFirstRightCol(y, x, sampleJson):                        # Returns y and x coordinate if there is a space available
    for i in range(x+1, 7):                                     # in the columns to the right, else, return (-1, -1) to go left
        for j in range(8):
            if checkDesc(j+1, i, sampleJson) == "UNUSED":
                return j+1, i
        return -1, -1

def findFirstLeftCol(y, x, sampleJson):                         # Returns y and x coordinate if there is a space available 
    for i in range(x-1, 0, -1):                                 # in the columns to the left, else, return (-1, -1) to go right 
        for j in range(8):
            if checkDesc(j+1, i, sampleJson) == "UNUSED":
                return j+1, i
    return -1, -1
    

#-----------------------------------------------------MAIN CODE---------------------------------------------------------------------------#

if __name__ == "__main__":
    with open('./shipCase1.json', 'r') as f:
        sampleJson = json.load(f)
    # print("THIS IS THE SAMPLE JSON AT THE BEGINNING OF ANY ITERATIONS")
    # print(sampleJson)
    print("Executing balance on case 1.")
    #Pass Json here
    balance(sampleJson)

    print("After balancing, it took us a total of " + str(moves) + " moves. It took a total amount of " + str(tot_distance) + " minutes to execute.")
    # print(moves, tot_distance)
    # print("The following are the start and end positions of each container on the ship:")
    # print(move_Dict)
    print("The final manifest looks like this:")
    print(sampleJson)
