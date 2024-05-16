# Copyright 2023 WATonomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from producer.producer_core import ProducerCore


def test_update_position():
    producer_core = ProducerCore(1, 1, 1, 1)
    producer_core.update_position()

    assert producer_core.serialize_data() == \
        "x:1.5773502691896257;y:1.5773502691896257;z:1.5773502691896257;"
