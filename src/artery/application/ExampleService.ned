//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple ExampleService like ItsG5Service
{
    parameters:
        // Parametrize ALI selection
        // 0: Load Balancing
        // 1: CLR-based Sequential Filling
        // 2: CALIS
        // 3: Time-based CASF
        // 4: Choose the ALI with the lowest CLR
        // 5: Choose the ALI with the shortest TRC
        // 6: CLR-based CASF
        int aliSelection = default(6);
        
        // At which CLR rate shall secondary channels be turned on
        double seqFillThreshold = default(0.50);

        // We consider a TRC of 20Hz and queue size of 5
        // Thus 0.050 * 5 plus a TRC is 300ms
        double casfThreshold @unit(s) = default(0.550s); 
        double queueFactor = default(0.8); 

        int messageSize = default(320);
        int tcPrimary = default(2);
        int tcAlternate = default(0);

        volatile double genRate = exponential(0.025);
}