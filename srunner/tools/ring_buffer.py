from threading import Lock


class RingBuffer:
    def __init__(self, size: int):
        self.ring = [[0, True, None] for _ in range(size)]
        self.size = size

        self.max_index = 0

    def get_frame(self, index: int, filter_not_completed: bool = True):
        real_index = index % self.size

        value = self.ring[real_index]
        if value[0] != index:
            return None

        if filter_not_completed:
            if value[1] == True:
                return None

        return value[2]

    def set_frame(self, index: int, value):
        self.max_index = max(self.max_index, index)
        if self.max_index - index >= self.size:
            return
        
        real_index = index % self.size

        self.ring[real_index] = [index, False, value]

    def set_frame_completed(self, index: int):
        real_index = index % self.size
        
        if self.ring[real_index][0] != index:
            return

        self.ring[real_index][1] = True

    def get_all_indexes(self, filter_not_completed: bool = True) -> list[int]:
        indexes = []

        for offset in range(0, self.size):
            index = self.max_index - offset
            real_index = index % self.size

            value = self.ring[real_index]
            if value[0] != index:
                continue

            if filter_not_completed:
                if value[1] == True:
                    continue

            indexes.append(index)

        return indexes

            
