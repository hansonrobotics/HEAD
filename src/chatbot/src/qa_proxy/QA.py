
class QA(object):

    def ask(self, *args, **kwargs):
        return NotImplemented

class Condition(object):

    def satisfied(self, **kwargs):
        return NotImplemented
    
class QAException(Exception): pass
    
class LanguageCondition(Condition):
    def __init__(self, lang):
        self.lang = lang

    def _is_lang_match(self, lang):
        return self.lang == lang

    def satisfied(self, lang):
        return self._is_lang_match(lang)

class ProxyQA(QA, Condition):
    def __init__(self, *args, **kwargs):
        super(ProxyQA, self).__init__(*args, **kwargs)
        self.conds = []

    def add_condition(self, cond):
        self.conds.append(cond)

    def satisfied(self, **kwargs):
        if kwargs:
            return all([c.satisfied(**kwargs) for c in self.conds])
        else:
            return False

    def ask(self, question, *args, **kwargs):
        if self.satisfied(**kwargs):
            return self._ask(question, *args, **kwargs)
        else:
            raise QAException("Condition is not satisfied")

    def _ask(self, question, *args, **kwargs):
        return NotImplemented
