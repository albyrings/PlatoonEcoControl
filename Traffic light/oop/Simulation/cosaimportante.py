
import random 

counter = 0
rang = 1000000
trial_list = []
for i in range(rang):
    cards = 52
    found = False
    trial = 0
    while not found:
        trial+=1
        card = random.randint(1, cards)
        if card == 1:
            trial_list.append(trial)
            found = True
        else:
            cards -= 1

print(sum(trial_list)/len(trial_list))