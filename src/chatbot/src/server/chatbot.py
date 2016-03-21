from characters import CHARACTERS

SUCCESS=0
WRONG_CHARACTER_NAME=1

def get_character(name):
    for character in CHARACTERS:
        if character.name == name:
            return character

def ask(name, question, session=None):
    character = get_character(name)
    if not character:
        return None, WRONG_CHARACTER_NAME
    answer = character.respond(question, session)
    if answer:
        return answer, SUCCESS
    else:
        generic = get_character('generic')
        generic.set_properties(character.get_properties())
        answer = generic.respond(question, session)
        return answer, SUCCESS

if __name__ == '__main__':
    for character in CHARACTERS:
        print ask(character.name, 'what is your name')

